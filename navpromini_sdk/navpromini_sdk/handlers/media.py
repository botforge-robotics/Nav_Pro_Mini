#!/usr/bin/env python3
"""Media handler: Image and video upload and management for robot displays."""

from __future__ import annotations

import os
import re
import time
import subprocess
from pathlib import Path

from navpromini_sdk.handlers.base import BaseHandler, ApiError

IMAGE_EXTENSIONS = {'.jpg', '.jpeg', '.png', '.gif', '.webp', '.bmp', '.svg'}
VIDEO_EXTENSIONS = {'.mp4', '.webm', '.mkv', '.mov', '.avi', '.ogv'}
ALL_MEDIA_EXTENSIONS = IMAGE_EXTENSIONS | VIDEO_EXTENSIONS


def get_media_dir() -> Path:
    """Return the active media directory, ensuring it exists and is writable."""
    candidates = []
    env_dir = os.environ.get('NAVPRO_MEDIA_DIR')
    if env_dir:
        candidates.append(Path(env_dir))

    user = os.environ.get('NAVPRO_USER') or os.environ.get('SUDO_USER')
    if user and user != 'root':
        candidates.append(Path(f'/home/{user}/media'))
        candidates.append(Path(f'/home/{user}/.navpromini/media'))

    candidates.append(Path('/home/navpromini/media'))
    candidates.append(Path('/home/navpromini/.navpromini/media'))
    candidates.append(Path.home() / 'media')
    candidates.append(Path.home() / '.navpromini' / 'media')

    for c in candidates:
        if c.exists() and c.is_dir():
            try:
                os.chmod(c, 0o777)
            except Exception:
                pass
            return c

    # Create primary candidate
    primary = candidates[0] if candidates else Path.home() / '.navpromini' / 'media'
    primary.mkdir(parents=True, exist_ok=True)
    try:
        os.chmod(primary, 0o777)
    except Exception:
        pass
    return primary


MEDIA_DIR = get_media_dir()


def _get_media_type(ext: str) -> str:
    ext = ext.lower()
    if ext in VIDEO_EXTENSIONS:
        return 'video'
    if ext in IMAGE_EXTENSIONS:
        return 'image'
    return 'unknown'


class MediaUploadHandler(BaseHandler):
    """POST /api/v1/media/upload - Accepts multipart file upload (images, videos)."""

    def post(self) -> None:
        if 'file' not in self.request.files:
            raise ApiError(400, 'missing_file', 'No file provided in form-data field "file"')

        target_dir = get_media_dir()
        uploaded_files = []
        host = self.request.host

        for file_obj in self.request.files['file']:
            raw_filename = file_obj.get('filename') or 'uploaded_media'
            safe_name = re.sub(r'[^a-zA-Z0-9_.-]', '_', raw_filename)
            ext = Path(safe_name).suffix.lower()
            if not ext:
                content_type = file_obj.get('content_type', '')
                if 'image/png' in content_type:
                    ext = '.png'
                elif 'image/jpeg' in content_type:
                    ext = '.jpg'
                elif 'video/mp4' in content_type:
                    ext = '.mp4'
                elif 'video/webm' in content_type:
                    ext = '.webm'
                else:
                    ext = '.bin'

            stem = Path(safe_name).stem
            timestamp = int(time.time())
            final_filename = f"{stem}_{timestamp}{ext}"
            dest_path = target_dir / final_filename

            with open(dest_path, 'wb') as f:
                f.write(file_obj['body'])

            try:
                os.chmod(dest_path, 0o666)
            except Exception:
                pass

            if ext in ('.mp4', '.mov', '.m4v'):
                # Optimize video container for smooth streaming & instant playback on robot screen
                tmp_opt = dest_path.with_suffix('.faststart.mp4')
                try:
                    res = subprocess.run(
                        ['ffmpeg', '-y', '-i', str(dest_path), '-c', 'copy', '-movflags', '+faststart', str(tmp_opt)],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10
                    )
                    if res.returncode == 0 and tmp_opt.exists() and tmp_opt.stat().st_size > 0:
                        tmp_opt.replace(dest_path)
                except Exception:
                    if tmp_opt.exists():
                        tmp_opt.unlink(missing_ok=True)

            media_type = _get_media_type(ext)
            media_url = f"http://{host}/media/{final_filename}"

            item = {
                'ok': True,
                'filename': final_filename,
                'name': raw_filename,
                'type': media_type,
                'url': media_url,
                'size': len(file_obj['body']),
                'content_type': file_obj.get('content_type', 'application/octet-stream'),
            }
            uploaded_files.append(item)
            self.bridge.emit_event('media.uploaded', item)

        if len(uploaded_files) == 1:
            self.send(uploaded_files[0], status=201)
        else:
            self.send({'ok': True, 'count': len(uploaded_files), 'files': uploaded_files}, status=201)


class MediaListHandler(BaseHandler):
    """GET /api/v1/media - Lists uploaded media files."""

    def get(self) -> None:
        target_dir = get_media_dir()
        files = []
        host = self.request.host

        scanned_dirs = [target_dir]
        for extra in [Path('/home/navpromini/media'), Path('/home/navpromini/.navpromini/media'), Path('/root/.navpromini/media')]:
            if extra.exists() and extra.is_dir() and extra.resolve() != target_dir.resolve():
                scanned_dirs.append(extra)

        seen_names = set()
        for d in scanned_dirs:
            try:
                for p in sorted(d.iterdir(), key=lambda x: x.stat().st_mtime, reverse=True):
                    if p.is_file() and not p.name.startswith('.') and p.name not in seen_names:
                        ext = p.suffix.lower()
                        media_type = _get_media_type(ext)
                        if media_type != 'unknown' or ext in ('.mp4', '.jpg', '.png', '.jpeg', '.gif', '.webm'):
                            seen_names.add(p.name)
                            # Create clean display name by stripping timestamp suffix if present
                            clean_name = re.sub(r'_\d{10}(\.[a-zA-Z0-9]+)$', r'\1', p.name)
                            files.append({
                                'filename': p.name,
                                'name': clean_name,
                                'type': media_type,
                                'url': f"http://{host}/media/{p.name}",
                                'size': p.stat().st_size,
                                'mtime': p.stat().st_mtime,
                                'ext': ext,
                            })
            except Exception:
                pass

        files.sort(key=lambda x: x['mtime'], reverse=True)
        self.send({'media': files, 'count': len(files)})


class MediaItemHandler(BaseHandler):
    """DELETE /api/v1/media/<filename> - Deletes an uploaded media file."""

    def delete(self, filename: str) -> None:
        filename = os.path.basename(filename.strip())
        if not filename or '/' in filename or '\\' in filename or filename.startswith('.'):
            raise ApiError(400, 'invalid_filename', 'Invalid filename specified')

        deleted = False
        target_dir = get_media_dir()
        candidate_dirs = [target_dir, Path('/home/navpromini/media'), Path('/home/navpromini/.navpromini/media'), Path('/root/.navpromini/media')]

        for d in candidate_dirs:
            if not d.exists():
                continue
            target_file = d / filename
            if target_file.is_file():
                try:
                    target_file.unlink()
                    deleted = True
                except Exception as exc:
                    raise ApiError(500, 'delete_failed', f'Failed to delete file: {exc}')

        if not deleted:
            raise ApiError(404, 'not_found', f'Media file {filename!r} not found')

        self.bridge.emit_event('media.deleted', {'filename': filename})
        self.send({'ok': True, 'deleted': filename})
