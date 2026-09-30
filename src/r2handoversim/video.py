"""Encode actual Isaac Sim viewport frames without optional Python packages."""
from pathlib import Path
import shutil
import subprocess
import math


def complete_png(path):
    """Kit's capture future may finish before its asynchronous file writer."""
    try:
        with Path(path).open("rb") as stream:
            stream.seek(-12, 2)
            return stream.read() == b"\x00\x00\x00\x00IEND\xaeB`\x82"
    except (OSError, ValueError):
        return False


def require_encoder():
    executable = shutil.which("ffmpeg")
    if executable is None:
        raise ValueError("MP4 recording requires ffmpeg on PATH; install FFmpeg, then retry --video")
    return executable


def encode_frames(directory, destination, count, dt_s, speed=1.):
    if count < 1 or not math.isfinite(dt_s) or dt_s <= 0 or not math.isfinite(speed) or speed <= 0:
        raise ValueError("Video needs frames, a positive timestep and a positive playback speed")
    directory, destination = Path(directory), Path(destination)
    for i in range(count):
        frame = directory / f"{i:06d}.png"
        if not frame.is_file() or frame.stat().st_size == 0:
            raise RuntimeError(f"Missing recorded viewport frame: {frame}")
    temporary = destination.with_name(destination.stem + ".partial.mp4")
    try:
        subprocess.run([require_encoder(), "-hide_banner", "-loglevel", "error", "-y",
            "-framerate", str(speed/dt_s), "-i", str(directory/"%06d.png"),
            "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2,tpad=start_duration=1:stop_duration=2:stop_mode=clone:start_mode=clone",
            "-r", "30", "-c:v", "libx264", "-preset", "fast", "-crf", "18",
            "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(temporary)],
            check=True, capture_output=True, timeout=180)
        if not temporary.is_file() or temporary.stat().st_size == 0:
            raise RuntimeError("FFmpeg produced no video")
        temporary.replace(destination)
    except subprocess.CalledProcessError as exc:
        raise RuntimeError(f"Video encoding failed: {exc.stderr.decode(errors='replace')[-2000:]}") from exc
    except subprocess.TimeoutExpired as exc:
        raise RuntimeError("Video encoding timed out after 180 seconds") from exc
    finally:
        temporary.unlink(missing_ok=True)
