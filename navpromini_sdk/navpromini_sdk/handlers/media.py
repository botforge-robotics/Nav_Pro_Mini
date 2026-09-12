#!/usr/bin/env python3
"""Media handler: Image and video upload and management for robot displays."""

from __future__ import annotations

import os
import re
import time
from pathlib import Path

from navpromini_sdk.handlers.base import BaseHandler, ApiError

MEDIA_DIR = Path.home() / '.navpromini' / 'media'
MEDIA_DIR.mkdir(parents=True, exist_ok=True)


class MediaUploadHandler(BaseHandler):
    """POST /api/v1/media/upload - Accepts multipart file upload (images, videos)."""

    def post(self) -> None:
        if 'file' not in self.request.files:
            raise ApiError(400, 'missing_file', 'No file provided in form-data field "file"')

        file_obj = self.request.files['file'][0]
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
            else:
                ext = '.bin'

        stem = Path(safe_name).stem
        timestamp = int(time.time())
        final_filename = f"{stem}_{timestamp}{ext}"
        dest_path = MEDIA_DIR / final_filename

        with open(dest_path, 'wb') as f:
            f.write(file_obj['body'])

        host = self.request.host
        media_url = f"http://{host}/media/{final_filename}"

        self.send({
            'ok': True,
            'filename': final_filename,
            'url': media_url,
            'size': len(file_obj['body']),
            'content_type': file_obj.get('content_type', 'application/octet-stream'),
        }, status=201)


class MediaListHandler(BaseHandler):
    """GET /api/v1/media - Lists uploaded media files."""

    def get(self) -> None:
        files = []
        host = self.request.host
        if MEDIA_DIR.exists():
            for p in sorted(MEDIA_DIR.iterdir(), key=lambda x: x.stat().st_mtime, reverse=True):
                if p.is_file() and not p.name.startswith('.'):
                    files.append({
                        'filename': p.name,
                        'url': f"http://{host}/media/{p.name}",
                        'size': p.stat().st_size,
                        'mtime': p.stat().st_mtime,
                    })
        self.send({'media': files})
