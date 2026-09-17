#!/usr/bin/env python3
"""Serve an image or stream a video as H.265 RTP to a camera port.

Usage examples:
  # Serve the hard-coded image (default) to a browser endpoint (HTTP):
  python3 scripts/serve_image.py --port 8081

  # Stream a YouTube video to the `zed` camera port (uses camera config to resolve IP/port):
  python3 scripts/serve_image.py --video-url "https://youtu.be/TKiHyhc9ooQ" --camera zed

  # Stream to a specific IP/port:
  python3 scripts/serve_image.py --video-url "https://youtu.be/TKiHyhc9ooQ" --target-ip 127.0.0.1 --target-port 8087

Notes:
- Requires `ffmpeg` and (recommended) `yt-dlp` installed.
"""
import argparse
import mimetypes
import os
import re
import sys
import subprocess
import shlex
import shutil
import tempfile
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.request import urlopen, Request
from urllib.error import URLError, HTTPError

try:
    import yaml
except Exception:
    yaml = None

HARDCODED_IMAGE_URL = (
    "https://encrypted-tbn3.gstatic.com/licensed-image?"
    "q=tbn:ANd9GcSdEIJy6_6UxkUJGaRAGocOe-sTYXFdIiNLFqg8KQFEMzvrkVFf8vym03JfW2Tswso7SlMUKKq1_snUprE"
)


def find_camera_info(camera_name, config_path):
    if not os.path.exists(config_path):
        return None, None
    if yaml:
        try:
            data = yaml.safe_load(open(config_path, "r"))
        except Exception:
            data = None
        if isinstance(data, dict):
            stack = [data]
            while stack:
                node = stack.pop()
                if isinstance(node, dict) and camera_name in node and isinstance(
                    node[camera_name], dict
                ):
                    cam = node[camera_name]
                    addr = cam.get("address")
                    port = cam.get("port")
                    try:
                        return (addr, int(port)) if port is not None else (addr, None)
                    except Exception:
                        return addr, None
                if isinstance(node, dict):
                    for v in node.values():
                        if isinstance(v, dict):
                            stack.append(v)
    # fallback: regex search
    text = open(config_path, "r").read()
    m_port = re.search(
        rf"{re.escape(camera_name)}:\s*(?:[^\n]*\n)+?\s*port:\s*(\d+)", text
    )
    m_addr = re.search(
        rf"{re.escape(camera_name)}:\s*(?:[^\n]*\n)+?\s*address:\s*([0-9a-zA-Z\.:_-]+)",
        text,
    )
    addr = m_addr.group(1) if m_addr else None
    port = int(m_port.group(1)) if m_port else None
    return addr, port


def resolve_with_ytdlp(url):
    ytdlp = shutil.which("yt-dlp") or shutil.which("youtube-dl")
    if not ytdlp:
        return url
    try:
        out = subprocess.check_output([ytdlp, "-f", "best", "-g", url], stderr=subprocess.DEVNULL)
        return out.decode("utf-8").strip()
    except Exception:
        return url


# determine whether we can copy instead of re-encode
def is_h265_input(url):
    # quick heuristic: local file extension or 'h265' in url
    lower = url.lower()
    if lower.endswith('.h265') or lower.endswith('.hevc') or '.h265' in lower or '.hevc' in lower:
        return True
    # try ffprobe if available and url is a file/path
    ffprobe = shutil.which('ffprobe')
    if ffprobe and os.path.exists(url):
        try:
            out = subprocess.check_output([ffprobe, '-v', 'error', '-select_streams', 'v:0',
                                           '-show_entries', 'stream=codec_name', '-of', 'default=nokey=1:noprint_wrappers=1', url],
                                          stderr=subprocess.DEVNULL)
            return b'hevc' in out.lower()
        except Exception:
            pass
    return False

sdp_path = None
if is_h265_input(url):
    # copy video (no re-encode) and write an SDP for receivers
    sdp_tf = tempfile.NamedTemporaryFile(delete=False, suffix='.sdp')
    sdp_path = sdp_tf.name
    sdp_tf.close()
    cmd = [
        ffmpeg, '-re', '-i', url,
        '-c:v', 'copy',
        '-f', 'rtp',
        '-payload_type', '96',
        'rtp://{}:{}'.format(target_ip, target_port),
        '-sdp_file', sdp_path
    ]
else:
    # re-encode path (existing behavior) but also write SDP with payload 96
    sdp_tf = tempfile.NamedTemporaryFile(delete=False, suffix='.sdp')
    sdp_path = sdp_tf.name
    sdp_tf.close()
    cmd = [
        ffmpeg, '-re', '-i', url,
        '-c:v', 'libx265', '-preset', 'fast', '-tune', 'zerolatency',
        '-b:v', bitrate, '-maxrate', bitrate, '-bufsize', str(int(bitrate.rstrip('k'))*2)+'k',
        '-f', 'rtp',
        '-payload_type', '96',
        'rtp://{}:{}'.format(target_ip, target_port),
        '-sdp_file', sdp_path
    ]
print('SDP written to', sdp_path)

class ImageHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self.send_response(200)
            self.send_header("Content-type", "text/html; charset=utf-8")
            self.end_headers()
            html = (
                '<!doctype html><html><head><meta charset="utf-8"><title>Image</title>'
                '<meta name="viewport" content="width=device-width,initial-scale=1">'
                "</head><body style='margin:0;padding:0;display:flex;align-items:center;justify-content:center;'>"
                '<img src="/image" style="max-width:100%;height:auto;" alt="served image">'
                "</body></html>"
            )
            self.wfile.write(html.encode("utf-8"))
            return
        if self.path == "/image":
            img = getattr(self.server, "image_path", None)
            if not img or not os.path.exists(img):
                self.send_error(404, "Image not found")
                return
            ctype = mimetypes.guess_type(img)[0] or "application/octet-stream"
            try:
                with open(img, "rb") as f:
                    data = f.read()
            except Exception:
                self.send_error(500, "Failed to read image")
                return
            self.send_response(200)
            self.send_header("Content-type", ctype)
            self.send_header("Content-length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
            return
        self.send_error(404, "Not found")


def download_image(url):
    try:
        req = Request(url, headers={"User-Agent": "python-urllib/3"})
        with urlopen(req) as resp:
            data = resp.read()
            ctype = resp.headers.get_content_type() if hasattr(resp.headers, "get_content_type") else resp.info().get_content_type()
            ext = mimetypes.guess_extension(ctype) or ".jpg"
            tf = tempfile.NamedTemporaryFile(delete=False, suffix=ext)
            tf.write(data)
            tf.close()
            return tf.name
    except (HTTPError, URLError) as e:
        print("Error: failed to download image:", e, file=sys.stderr)
        return None


def main():
    parser = argparse.ArgumentParser(description="Serve an image or stream video as H.265 RTP")
    parser.add_argument("--image", "-i", help="Path to the image file to serve (if omitted, uses the hard-coded URL)")
    parser.add_argument("--port", "-p", type=int, help="Port to listen on for the HTTP image viewer")
    parser.add_argument("--camera", "-c", help='Camera name from config/cameras.yaml to use its port (e.g. "zed")')
    parser.add_argument("--config", default=os.path.join(os.path.dirname(__file__), "..", "config", "cameras.yaml"),
                        help="Path to cameras.yaml (default: repo config/cameras.yaml)")
    parser.add_argument("--video-url", help="URL (or local path) to video to stream as H265 RTP")
    parser.add_argument("--target-ip", help="Target IP to send RTP to (overrides camera config)")
    parser.add_argument("--target-port", type=int, help="Target port to send RTP to (overrides camera config)")
    parser.add_argument("--bitrate", default="1000k", help="Bitrate for H265 encoder (e.g. 1000k)")
    args = parser.parse_args()

    image_path = None
    if args.image:
        image_path = os.path.expanduser(args.image)
        if not os.path.exists(image_path):
            print("Error: image not found:", image_path, file=sys.stderr)
            sys.exit(2)
    else:
        print("No --image provided; downloading hard-coded image...")
        image_path = download_image(HARDCODED_IMAGE_URL)
        if not image_path:
            print("Error: could not download hard-coded image", file=sys.stderr)
            sys.exit(2)

    # If user requested a video stream, resolve the target IP/port (from args or camera config)
    if args.video_url:
        targ_ip = args.target_ip
        targ_port = args.target_port
        if args.camera and (not targ_ip or not targ_port):
            cam_addr, cam_port = find_camera_info(args.camera, args.config)
            if not targ_ip:
                targ_ip = cam_addr or "127.0.0.1"
            if not targ_port:
                targ_port = cam_port
        if not targ_port:
            print("Error: no target port resolved for streaming (use --target-port or --camera)", file=sys.stderr)
            sys.exit(2)
        # launch ffmpeg streamer
        proc = stream_url_as_h265_rtp(args.video_url, targ_ip or "127.0.0.1", targ_port, bitrate=args.bitrate)
        if not proc:
            sys.exit(2)
        try:
            proc.wait()
        except KeyboardInterrupt:
            print("\nStopping ffmpeg...")
            try:
                proc.terminate()
                proc.wait(timeout=5)
            except Exception:
                proc.kill()
        return

    # Otherwise, run the simple HTTP server to show the image (optional)
    port = args.port or 8080
    server_address = ("", port)
    httpd = HTTPServer(server_address, ImageHandler)
    httpd.image_path = image_path
    print(f"Serving {image_path} on http://0.0.0.0:{port}/")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down")
        httpd.server_close()


if __name__ == "__main__":
    main()