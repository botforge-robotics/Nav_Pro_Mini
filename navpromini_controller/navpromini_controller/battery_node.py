#!/usr/bin/env python3
"""Publish Daly Smart BMS telemetry on ROS 2 topics.

Default port: /dev/battery_bms (udev symlink for FTDI USB–RS485).
"""

from __future__ import annotations

import json
from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32, Float32MultiArray, String

from navpromini_controller.daly_bms import DalySerial, DalySnapshot


class BatteryNode(Node):
    def __init__(self) -> None:
        super().__init__('navpromini_battery')

        self.declare_parameter('serial_port', '/dev/battery_bms')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('rate_hz', 2.0)
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('protocol', 'auto')  # auto | a5 | modbus
        self.declare_parameter('nominal_capacity_ah', 0.0)

        self._port = str(self.get_parameter('serial_port').value)
        self._baud = int(self.get_parameter('baudrate').value)
        rate = float(self.get_parameter('rate_hz').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._protocol_pref = str(self.get_parameter('protocol').value).lower()
        self._nominal_ah = float(self.get_parameter('nominal_capacity_ah').value)

        self._pub_state = self.create_publisher(BatteryState, 'battery/state', 10)
        self._pub_cells = self.create_publisher(Float32MultiArray, 'battery/cells', 10)
        self._pub_temps = self.create_publisher(Float32MultiArray, 'battery/temperatures', 10)
        self._pub_soc = self.create_publisher(Float32, 'battery/soc', 10)
        self._pub_voltage = self.create_publisher(Float32, 'battery/voltage', 10)
        self._pub_current = self.create_publisher(Float32, 'battery/current', 10)
        self._pub_json = self.create_publisher(String, 'battery/info', 10)
        self._pub_diag = self.create_publisher(DiagnosticArray, 'diagnostics', 10)

        self._bms = DalySerial(self._port, self._baud)
        self._ok = False
        try:
            self._bms.open()
            if self._protocol_pref == 'a5':
                if not self._bms.probe_a5():
                    raise RuntimeError('A5 protocol probe failed')
            elif self._protocol_pref == 'modbus':
                if not self._bms.probe_modbus():
                    raise RuntimeError('Modbus protocol probe failed')
            else:
                proto = self._bms.probe()
                self.get_logger().info(f'Daly BMS detected via {proto} on {self._port}')
            self._ok = True
        except Exception as exc:  # noqa: BLE001 — report and keep node alive
            self.get_logger().error(f'BMS open/probe failed: {exc}')

        period = 1.0 / max(rate, 0.2)
        self.create_timer(period, self._tick)
        self.get_logger().info(
            f'Battery node: port={self._port} baud={self._baud} '
            f'protocol={self._bms.protocol or self._protocol_pref}'
        )

    def destroy_node(self) -> bool:
        try:
            self._bms.close()
        except Exception:  # noqa: BLE001
            pass
        return super().destroy_node()

    def _tick(self) -> None:
        if not self._ok:
            return
        try:
            snap = self._bms.read()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'BMS read failed: {exc}', throttle_duration_sec=5.0)
            return
        self._publish(snap)

    def _publish(self, snap: DalySnapshot) -> None:
        now = self.get_clock().now().to_msg()

        msg = BatteryState()
        msg.header.stamp = now
        msg.header.frame_id = self._frame_id
        msg.voltage = float(snap.pack_voltage_v)
        msg.current = float(snap.pack_current_a)
        if snap.soc_percent == snap.soc_percent:
            msg.percentage = max(0.0, min(1.0, snap.soc_percent / 100.0))
        else:
            msg.percentage = float('nan')
        msg.charge = float(snap.remain_capacity_ah)
        capacity = self._nominal_ah if self._nominal_ah > 0 else float('nan')
        if snap.remain_capacity_ah == snap.remain_capacity_ah and snap.soc_percent > 1.0:
            # Estimate full capacity from remain / SOC
            capacity = snap.remain_capacity_ah / (snap.soc_percent / 100.0)
        msg.capacity = capacity
        msg.design_capacity = self._nominal_ah if self._nominal_ah > 0 else capacity
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.present = True

        if snap.charge_state == 1 or snap.pack_current_a > 0.2:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        elif snap.charge_state == 2 or snap.pack_current_a < -0.2:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        elif snap.soc_percent >= 99.0:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
        else:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING

        if snap.failure_bits:
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNSPEC_FAILURE
        elif snap.max_temp_c == snap.max_temp_c and snap.max_temp_c > 55.0:
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_OVERHEAT
        elif snap.min_cell_v == snap.min_cell_v and snap.min_cell_v < 2.5:
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_DEAD
        else:
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD

        msg.cell_voltage = [float(v) for v in snap.cell_voltages_v]
        if snap.temperatures_c:
            # BatteryState has cell_temperature in newer msgs; Jazzy has it
            if hasattr(msg, 'cell_temperature'):
                msg.cell_temperature = [float(t) for t in snap.temperatures_c]

        self._pub_state.publish(msg)

        cells = Float32MultiArray()
        cells.data = [float(v) for v in snap.cell_voltages_v]
        self._pub_cells.publish(cells)

        temps = Float32MultiArray()
        temps.data = [float(t) for t in snap.temperatures_c]
        self._pub_temps.publish(temps)

        if snap.soc_percent == snap.soc_percent:
            self._pub_soc.publish(Float32(data=float(snap.soc_percent)))
        if snap.pack_voltage_v == snap.pack_voltage_v:
            self._pub_voltage.publish(Float32(data=float(snap.pack_voltage_v)))
        if snap.pack_current_a == snap.pack_current_a:
            self._pub_current.publish(Float32(data=float(snap.pack_current_a)))

        charge_name = {0: 'idle', 1: 'charging', 2: 'discharging'}.get(
            snap.charge_state, f'unknown({snap.charge_state})'
        )
        info = {
            'protocol': snap.protocol,
            'pack_voltage_v': snap.pack_voltage_v,
            'pack_current_a': snap.pack_current_a,
            'soc_percent': snap.soc_percent,
            'remain_capacity_ah': snap.remain_capacity_ah,
            'cell_count': snap.cell_count,
            'temp_count': snap.temp_count,
            'cell_voltages_v': snap.cell_voltages_v,
            'temperatures_c': snap.temperatures_c,
            'max_cell_v': snap.max_cell_v,
            'min_cell_v': snap.min_cell_v,
            'max_cell_index': snap.max_cell_index,
            'min_cell_index': snap.min_cell_index,
            'max_temp_c': snap.max_temp_c,
            'min_temp_c': snap.min_temp_c,
            'charge_state': charge_name,
            'charge_mos_on': snap.charge_mos_on,
            'discharge_mos_on': snap.discharge_mos_on,
            'charger_connected': snap.charger_connected,
            'load_connected': snap.load_connected,
            'bms_life': snap.bms_life,
            'failure_bits': snap.failure_bits,
            'balance_bits': snap.balance_bits,
            'notes': snap.raw_notes,
        }
        self._pub_json.publish(String(data=json.dumps(info)))

        diag = DiagnosticArray()
        diag.header.stamp = now
        st = DiagnosticStatus()
        st.name = 'navpromini_battery/daly_bms'
        st.hardware_id = self._port
        st.level = DiagnosticStatus.ERROR if snap.failure_bits else DiagnosticStatus.OK
        st.message = f'{charge_name} SOC={snap.soc_percent:.1f}% V={snap.pack_voltage_v:.2f} I={snap.pack_current_a:.2f}A'
        for key, val in info.items():
            if key in ('cell_voltages_v', 'temperatures_c'):
                continue
            st.values.append(KeyValue(key=key, value=str(val)))
        diag.status.append(st)
        self._pub_diag.publish(diag)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BatteryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
