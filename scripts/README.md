# Utility Scripts

This directory contains utility scripts for system setup, testing, and debugging.

## Scripts

### launch_with_dds.sh
**DDS Environment Setup and Launch**

Configures DDS environment variables and launches ROS2 nodes with proper network settings.

**Usage**:
```bash
./scripts/launch_with_dds.sh
```

**What it does**:
1. Sets `ROS_DOMAIN_ID=0`
2. Configures `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
3. Points to `config/cyclonedds_jetson.xml`
4. Sets `ROS_LOCALHOST_ONLY=0` for network communication
5. Sources ROS2 workspace

**When to use**:
- Multi-machine communication
- Network setup for remote visualization
- Resolving DDS connection issues

---

### test_urg_connection.sh
**LiDAR Connection Diagnostic**

Comprehensive diagnostic tool for Hokuyo URG-04LX LiDAR connectivity.

**Usage**:
```bash
./scripts/test_urg_connection.sh
```

**What it checks**:
1. ✓ Network connectivity to `192.168.1.111`
2. ✓ Port `10940` accessibility
3. ✓ Host network configuration
4. ✓ Firewall rules
5. ✓ LiDAR response

**Common Issues Detected**:
- Host not on `192.168.1.x` subnet
- Firewall blocking port 10940
- LiDAR power/connection issues
- Network cable problems

**Output Example**:
```
=== Hokuyo LiDAR Connection Test ===
✓ Ping successful
✓ Port 10940 accessible
✓ Host IP correctly configured
✗ LiDAR not responding - check power
```

---

## Adding New Scripts

When adding utility scripts:
1. Place script in this directory
2. Make executable: `chmod +x scripts/script_name.sh`
3. Add usage documentation here
4. Follow naming convention: `action_description.sh`
5. Add shebang: `#!/bin/bash`
6. Include error handling

**Script Template**:
```bash
#!/bin/bash
# Description: What this script does
# Usage: ./scripts/script_name.sh [args]

set -e  # Exit on error

# Script logic here
```

---

## Script Categories

### Setup Scripts
- `launch_with_dds.sh` - DDS environment configuration

### Diagnostic Scripts
- `test_urg_connection.sh` - LiDAR connectivity test

### Build Scripts
*(Add build automation scripts here)*

### Deployment Scripts
*(Add deployment scripts here)*

---

**Related Documentation**:
- LiDAR setup: `docs/index/PROJECT_INDEX.md#lidar-connection-issues`
- DDS configuration: `config/README.md`
- Quick commands: `docs/index/QUICK_REFERENCE.md`
