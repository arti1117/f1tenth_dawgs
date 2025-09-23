# ├─ master_nx/
# │  ├─ master_zmq.py               # NX에서 실행: CMA-ES + ZeroMQ 분산
# │  └─ config.py                   # 마스터 설정 (IP/포트/도메인)

MASTER_BIND_IP = "0.0.0.0"  # NX에서 외부 워커 접속 허용
JOB_PORT = 5555             # PUSH bind
RESULT_PORT = 5556          # PULL bind
PARAMS_TEMPLATE = "../shared/params_template.yaml"

# (옵션) NX 로컬에서 worker도 띄울 경우, 로컬 도메인들
LOCAL_DOMAINS = [1, 2]      # NX에서 돌릴 ROS_DOMAIN_ID (없으면 빈 리스트)


