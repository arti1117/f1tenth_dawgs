# F1TENTH DAWGS - Documentation Index

**Generated comprehensive documentation for the F1TENTH DAWGS autonomous racing system**

---

## 📚 Documentation Structure

This directory contains complete reference documentation for the F1TENTH DAWGS project, organized as follows:

### Core Documentation Files

1. **[PROJECT_INDEX.md](PROJECT_INDEX.md)** - Complete project overview
   - System architecture and data flow
   - 37 ROS2 packages organized by category
   - Launch files and configuration
   - Development guides and troubleshooting
   - **Start here for system understanding**

2. **[PACKAGE_REFERENCE.md](PACKAGE_REFERENCE.md)** - Detailed package documentation
   - Complete reference for all 37 packages
   - Message types, topics, parameters
   - Configuration files and usage examples
   - **Use for package-specific details**

3. **[ROS2_INTERFACES.md](ROS2_INTERFACES.md)** - ROS2 communication reference
   - Topic architecture and message flows
   - Complete topic reference with QoS policies
   - TF tree and frame descriptions
   - Service and parameter documentation
   - **Reference for ROS2 integration**

4. **[QUICK_REFERENCE.md](QUICK_REFERENCE.md)** - Fast lookup guide
   - Quick start workflows
   - Common commands and debugging
   - Troubleshooting solutions
   - Performance tuning cheatsheet
   - **Use for daily development**

---

## 🎯 Quick Navigation by Task

### I want to...

**Understand the system**:
→ Start with [PROJECT_INDEX.md](PROJECT_INDEX.md) - System Overview section

**Learn about a specific package**:
→ See [PACKAGE_REFERENCE.md](PACKAGE_REFERENCE.md) - Find package by category

**Work with ROS2 topics**:
→ Check [ROS2_INTERFACES.md](ROS2_INTERFACES.md) - Core Topics Reference

**Get started quickly**:
→ Follow [QUICK_REFERENCE.md](QUICK_REFERENCE.md) - Quick Start Workflows

**Debug an issue**:
→ See [QUICK_REFERENCE.md](QUICK_REFERENCE.md) - Common Issues & Fixes

**Tune performance**:
→ Check [QUICK_REFERENCE.md](QUICK_REFERENCE.md) - Performance Tuning

**Configure parameters**:
→ See [PACKAGE_REFERENCE.md](PACKAGE_REFERENCE.md) - Configuration sections

---

## 📋 Documentation Coverage

### Project Statistics
- **Total Packages**: 37 ROS2 packages
- **Launch Files**: 37 launch configurations
- **Config Files**: 137 YAML/Lua configurations
- **Topics Documented**: 30+ core topics
- **Services**: 10+ services
- **Parameters**: 100+ configuration parameters

### Key Components Documented
✅ Agent Integration (agent_dawgs)
✅ Hardware Drivers (13 packages)
✅ Controllers (7 packages)
✅ Localization (particle_filter, SLAM)
✅ Planning (path_planner, Frenet system)
✅ Control (path_tracker, pure_pursuit)
✅ Utilities (14 packages)

---

## 🗺️ Documentation Map

```
Documentation Hierarchy:

PROJECT_INDEX.md (Top-level overview)
├─ System Architecture
├─ Package Organization ────────→ PACKAGE_REFERENCE.md (Package details)
│                                  ├─ Agent Integration
│                                  ├─ Base System
│                                  ├─ Controllers
│                                  ├─ Peripherals
│                                  └─ Utilities
│
├─ ROS2 Topics ─────────────────→ ROS2_INTERFACES.md (Communication)
│                                  ├─ Topic Architecture
│                                  ├─ Message Types
│                                  ├─ TF Tree
│                                  └─ QoS Policies
│
└─ Development Guides ──────────→ QUICK_REFERENCE.md (Daily use)
                                   ├─ Quick Start
                                   ├─ Build Commands
                                   ├─ Debugging
                                   └─ Troubleshooting
```

---

## 🔍 Search by Keyword

### Architecture & Design
- System architecture → [PROJECT_INDEX.md](PROJECT_INDEX.md#system-overview)
- Data flow → [PROJECT_INDEX.md](PROJECT_INDEX.md#real-time-data-flow-example-lap)
- Frenet planning → [PROJECT_INDEX.md](PROJECT_INDEX.md#frenet-coordinate-system-key-innovation)
- Two-stage planner → [PACKAGE_REFERENCE.md](PACKAGE_REFERENCE.md#path_planner--primary)

### Hardware & Drivers
- VESC motor controller → [PACKAGE_REFERENCE.md](PACKAGE_REFERENCE.md#vesc_driver)
- LiDAR (Hokuyo) → [ROS2_INTERFACES.md](ROS2_INTERFACES.md#scan)
- IMU (iAHRS) → [PACKAGE_REFERENCE.md](PACKAGE_REFERENCE.md#iahrs_driver)
- Joystick (F-710) → [PACKAGE_REFERENCE.md](PACKAGE_REFERENCE.md#joy_teleop)
- Ackermann mux → [PACKAGE_REFERENCE.md](PACKAGE_REFERENCE.md#ackermann_mux)

### Planning & Control
- Path planner config → [PACKAGE_REFERENCE.md](PACKAGE_REFERENCE.md#path_planner--primary)
- Lattice sampling → [PROJECT_INDEX.md](PROJECT_INDEX.md#real-time-data-flow-example-lap)
- Pure pursuit → [PACKAGE_REFERENCE.md](PACKAGE_REFERENCE.md#path_tracker)
- Gap following → [PACKAGE_REFERENCE.md](PACKAGE_REFERENCE.md#gap_follow)

### Localization & Mapping
- Particle filter → [PACKAGE_REFERENCE.md](PACKAGE_REFERENCE.md#particle_filter)
- SLAM (Cartographer) → [PROJECT_INDEX.md](PROJECT_INDEX.md#launch-files)
- Map format → [PACKAGE_REFERENCE.md](PACKAGE_REFERENCE.md#maps)
- TF frames → [ROS2_INTERFACES.md](ROS2_INTERFACES.md#tf-tree)

### ROS2 Communication
- Topic list → [ROS2_INTERFACES.md](ROS2_INTERFACES.md#core-topics-reference)
- QoS policies → [ROS2_INTERFACES.md](ROS2_INTERFACES.md#qos-policies)
- Message types → [ROS2_INTERFACES.md](ROS2_INTERFACES.md#message-type-reference)
- Services → [ROS2_INTERFACES.md](ROS2_INTERFACES.md#services)

### Development
- Build commands → [QUICK_REFERENCE.md](QUICK_REFERENCE.md#-build-commands)
- Debugging → [QUICK_REFERENCE.md](QUICK_REFERENCE.md#-debugging-tools)
- Common issues → [QUICK_REFERENCE.md](QUICK_REFERENCE.md#-common-issues--fixes)
- Performance tuning → [QUICK_REFERENCE.md](QUICK_REFERENCE.md#-performance-tuning-cheatsheet)

---

## 🚀 Quick Start Paths

### New Developer Path
1. Read [PROJECT_INDEX.md](PROJECT_INDEX.md) - System Overview (10 min)
2. Follow [QUICK_REFERENCE.md](QUICK_REFERENCE.md) - Quick Start Workflows (5 min)
3. Reference [PACKAGE_REFERENCE.md](PACKAGE_REFERENCE.md) as needed

### Debugging Path
1. Check [QUICK_REFERENCE.md](QUICK_REFERENCE.md) - Common Issues (2 min)
2. Use [ROS2_INTERFACES.md](ROS2_INTERFACES.md) - Topic Debugging (5 min)
3. Enable verbose logging per [QUICK_REFERENCE.md](QUICK_REFERENCE.md) - Debugging Tools

### Integration Path
1. Study [ROS2_INTERFACES.md](ROS2_INTERFACES.md) - Topic Architecture (15 min)
2. Review [PACKAGE_REFERENCE.md](PACKAGE_REFERENCE.md) - Package details (20 min)
3. Reference [PROJECT_INDEX.md](PROJECT_INDEX.md) - Configuration Files (10 min)

---

## 📝 Documentation Standards

### File Naming Convention
- `PROJECT_INDEX.md` - Top-level project overview
- `PACKAGE_REFERENCE.md` - Package-specific documentation
- `ROS2_INTERFACES.md` - Communication interfaces
- `QUICK_REFERENCE.md` - Fast lookup guide

### Cross-Referencing
All documents extensively cross-reference each other using relative links. Follow links to navigate between related topics.

### Update Policy
Documentation generated: 2025-10-31
Update frequency: As project evolves
Update trigger: Major system changes, new packages, API changes

---

## 🔗 External Resources

### Primary Project Guide
- `/home/arti/Documents/RoboRacer/CLAUDE.md` - Root project guide for Claude Code

### Additional Documentation
- `src/controller/CONTROLLER.md` - Controller change history
- `src/agent_dawgs/README.md` - Agent configuration notes
- Package READMEs - Individual package documentation

### ROS2 Resources
- [ROS2 Foxy Documentation](https://docs.ros.org/en/foxy/)
- [F1TENTH Documentation](https://f1tenth.org/)
- [Cartographer ROS](https://google-cartographer-ros.readthedocs.io/)

---

## 💡 Tips for Using This Documentation

### Best Practices
1. **Start broad, then specific**: Begin with PROJECT_INDEX, drill down to PACKAGE_REFERENCE
2. **Use search**: Ctrl+F in files or use "Search by Keyword" above
3. **Follow cross-references**: Links connect related information
4. **Keep QUICK_REFERENCE handy**: Bookmark for daily commands

### When to Use Which File
- **Learning the system**: PROJECT_INDEX.md
- **Implementing a feature**: PACKAGE_REFERENCE.md + ROS2_INTERFACES.md
- **Debugging**: QUICK_REFERENCE.md first, then dive deeper
- **Daily development**: QUICK_REFERENCE.md

---

## 📞 Getting Help

### Documentation Issues
If documentation is unclear, outdated, or missing information:
1. Check related sections via cross-references
2. Refer to source code for definitive behavior
3. Update documentation with findings

### Technical Issues
1. Check [QUICK_REFERENCE.md](QUICK_REFERENCE.md) - Common Issues
2. Enable verbose logging for detailed diagnostics
3. Review ROS2 topic flows in [ROS2_INTERFACES.md](ROS2_INTERFACES.md)

---

## 🎓 Learning Path Recommendations

### Beginner (New to F1TENTH)
**Week 1**: System Understanding
- Day 1-2: [PROJECT_INDEX.md](PROJECT_INDEX.md) - Full read
- Day 3-4: [QUICK_REFERENCE.md](QUICK_REFERENCE.md) - Run simulation
- Day 5: Experiment with parameters

**Week 2**: Deep Dive
- Day 1-3: [PACKAGE_REFERENCE.md](PACKAGE_REFERENCE.md) - Core packages
- Day 4-5: [ROS2_INTERFACES.md](ROS2_INTERFACES.md) - Topic monitoring

### Intermediate (ROS2 Experience)
**Focus Areas**:
1. Frenet planning system ([PROJECT_INDEX.md](PROJECT_INDEX.md#frenet-coordinate-system-key-innovation))
2. Path planner configuration ([PACKAGE_REFERENCE.md](PACKAGE_REFERENCE.md#path_planner--primary))
3. Performance optimization ([QUICK_REFERENCE.md](QUICK_REFERENCE.md#-performance-tuning-cheatsheet))

### Advanced (System Integration)
**Focus Areas**:
1. Complete ROS2 interface understanding ([ROS2_INTERFACES.md](ROS2_INTERFACES.md))
2. Custom controller development (use [PACKAGE_REFERENCE.md](PACKAGE_REFERENCE.md) as template)
3. System-level optimization (cross-reference all docs)

---

**Generated**: 2025-10-31
**Documentation Version**: 1.0
**Project**: F1TENTH DAWGS Autonomous Racing System
**ROS2 Version**: Foxy

---

**For Claude Code Development**: See `/home/arti/Documents/RoboRacer/CLAUDE.md`
