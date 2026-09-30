# Showcase recordings

All clips are viewport recordings from Isaac Sim using the original local
UR5e/Robotiq USD, object meshes and visible MANO receiving hands. The lab preset
uses five area lights; final recordings use path tracing.

| Media | Scene | Presentation |
|---|---|---|
| `handover.gif` / `handover.mp4` | Screwdriver, left receiver, FS; all five checks passed | Synchronized workspace and right-oblique views |
| `receivers.gif` / `receivers.mp4` | Can and screwdriver, left/right receivers, A2, seed 27 | Four independent trials, each at 0.5× playback; completed clips hold their final frame |
| `cameras.jpg` / `multiview.mp4` | Same FS screwdriver trajectory as the hero | Overview, left, right and elevated cameras |

[Recording metadata](recordings.json) lists the evaluated montage outcomes.
These scenes demonstrate the workflow; paper-reference aggregate records are
available through the separate `paper-replay` command.

The GIFs provide inline GitHub previews. Their image links open H.264 MP4
attachments on the benchmark release. Source models and scene inputs are
configured locally under the [external asset terms](../../THIRD_PARTY.md).

To create your own clips, follow [quickstart](../quickstart.md), then replay a
resolved scene with the [camera presets](../rendering.md). `verify-output`
checks the run before `review-video` adds result captions.
