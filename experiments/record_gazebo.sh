#!/usr/bin/env bash
# Record a Gazebo run with the fixed scene camera of worlds/recording.world and
# encode it to MP4 (2x speed) and a README-sized GIF.
#
#   experiments/record_gazebo.sh NAME CONTROLLER SAFETY_FILTER 'REFERENCE_JSON' 'X,Y,YAW' OBSTACLE_SEED N_OBSTACLES
#   experiments/record_gazebo.sh nmpc_fig8 nmpc false '{"type": "figure8"}' '0.15,-0.1,0.3' 3 2
#
# Runs headless on its own Gazebo master port / ROS domain, so it does not
# interfere with an interactive session. Output: media/NAME.mp4, media/NAME.gif
set -euo pipefail
NAME=$1 CTRL=$2 SF=$3 REF=$4 OFFSET=$5 SEED=$6 NOBS=$7
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
WS=${TB3_WS:-$HOME/ar521_ws}
FRAMES=/tmp/tb3_mpc_frames
mkdir -p "$ROOT/media"
rm -rf "$FRAMES"

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
source /usr/share/gazebo/setup.sh
set -u
export GAZEBO_MASTER_URI=http://localhost:${GZ_PORT:-11399}
export ROS_DOMAIN_ID=${REC_DOMAIN:-88}

ros2 launch tb3_mpc sim.launch.py gui:=false rviz:=false exit_on_finish:=true \
  world:="$(ros2 pkg prefix tb3_mpc)/share/tb3_mpc/worlds/recording.world" \
  controller:="$CTRL" safety_filter:="$SF" reference:="$REF" ref_offset:="$OFFSET" \
  obstacle_seed:="$SEED" n_obstacles:="$NOBS" run_name:="rec_$NAME" \
  log_dir:="$ROOT/results/gazebo" > "/tmp/rec_$NAME.log" 2>&1 || true

N=$(ls "$FRAMES" 2>/dev/null | wc -l)
echo "[$NAME] $N frames"
[ "$N" -gt 10 ] || { echo "no frames recorded; see /tmp/rec_$NAME.log"; exit 1; }
# frames are numbered; feed them in order at 15 fps sim time, 2x speed
ls "$FRAMES"/*.jpg | sort -V | sed "s/^/file '/; s/$/'/" > /tmp/rec_list.txt
ffmpeg -loglevel error -y -f concat -safe 0 -r 15 -i /tmp/rec_list.txt \
  -vf "setpts=0.5*PTS,fps=30,format=yuv420p" -c:v libx264 -crf 22 "$ROOT/media/$NAME.mp4"
ffmpeg -loglevel error -y -i "$ROOT/media/$NAME.mp4" \
  -vf "fps=12,scale=640:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=128[p];[b][p]paletteuse=dither=bayer:bayer_scale=4" \
  "$ROOT/media/$NAME.gif"
ls -la "$ROOT/media/$NAME".*
