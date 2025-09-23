#!/usr/bin/env bash
set -e
MASTER_IP=${1:-192.168.0.8}  # NX IP
JOB_PORT=${2:-5555}
RES_PORT=${3:-5556}

# 병렬 도메인 (AGX는 여유가 크니 8개 예시)
for DID in 11 12 13 14 15 16 17 18; do
  echo "Launching worker domain $DID"
  # 각 워커는 별도 터미널/세션에서 실행하거나, tmux/screen 추천
  ROS_DOMAIN_ID=$DID python3 worker_agent.py \
    --master_ip $MASTER_IP --job_port $JOB_PORT --res_port $RES_PORT \
    --domain_id $DID &
done
echo "Started AGX workers (domains 11..18)."
