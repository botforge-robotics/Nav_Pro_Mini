# NavProMini local Open-RMF API server config (loaded via RMF_API_SERVER_CONFIG).
# Uses a dedicated sqlite DB so stale demo door rows do not crash startup.

from os.path import expanduser
import os

from api_server.default_config import config

run_dir = expanduser('~/.cache/navpromini_rmf_api')
os.makedirs(f'{run_dir}/cache', exist_ok=True)

use_sim = os.environ.get('RMF_API_USE_SIM_TIME', 'false').lower() in (
    '1', 'true', 'yes',
)
port = int(os.environ.get('RMF_API_SERVER_PORT', '8000'))
public_url = os.environ.get('RMF_API_SERVER_PUBLIC_URL', f'http://localhost:{port}')

config.update(
    {
        'host': os.environ.get('RMF_API_SERVER_HOST', '127.0.0.1'),
        'port': port,
        'db_url': f'sqlite://{run_dir}/db.sqlite3',
        'cache_directory': f'{run_dir}/cache',
        'public_url': public_url,
        'ros_args': ['-p', f'use_sim_time:={str(use_sim).lower()}'],
        'log_level': os.environ.get('RMF_API_SERVER_LOG_LEVEL', 'INFO'),
        'timezone': os.environ.get('RMF_API_SERVER_TIMEZONE', 'Asia/Kolkata'),
        'builtin_admin': 'admin',
    }
)
