# Configuration Files

This directory contains system-level configuration files for the F1TENTH DAWGS project.

## Files

### cyclonedds_jetson.xml
**DDS Communication Configuration**

CycloneDDS configuration for ROS2 inter-process and multi-machine communication.

**Key Settings**:
- **NetworkInterfaceAddress**: auto (auto-detects active interface)
- **AllowMulticast**: true (enables discovery across network)
- **MaxMessageSize**: 65500B (handles large LiDAR messages)
- **MinimumSocketReceiveBufferSize**: 10MB (for high-frequency sensor data)

**Usage**:
```bash
# Automatic (via launch script)
./scripts/launch_with_dds.sh

# Manual
export CYCLONEDDS_URI=file://$PWD/config/cyclonedds_jetson.xml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

**When to Use**:
- Multi-machine setups (e.g., Jetson + laptop)
- Network communication issues
- Large message handling (LiDAR, maps)

**Troubleshooting**:
- Check `cyclonedds.log` in project root for connection issues
- Verify all machines use same `ROS_DOMAIN_ID`
- Ensure network allows UDP multicast

---

## Adding New Configurations

When adding new system-level config files:
1. Place file in this directory
2. Document in this README
3. Reference in main `CLAUDE.md` if critical
4. Update launch scripts if needed

---

**Related Documentation**:
- Network setup: `docs/index/PROJECT_INDEX.md#network-configuration`
- Troubleshooting: `docs/index/QUICK_REFERENCE.md#dds-communication-issues`
