<div align="center">

<picture>
    <source media="(prefers-color-scheme: light)" srcset="images/logo/steelhawks-light-mode.svg">
    <img src="images/logo/steelhawks-dark-mode.svg" alt="Steel Hawks Logo">
</picture>

# Shaquille O'Steel
### FRC Team Steel Hawks 2601 :: 2026 Robot Code

[![WPILib](https://img.shields.io/badge/WPILib-v2026.2.1-AC2B37?style=flat-square&logo=github)](https://github.com/wpilibsuite/allwpilib/releases/tag/v2026.2.1)
[![Java](https://img.shields.io/badge/Java-17-ED8B00?style=flat-square&logo=openjdk&logoColor=white)](https://www.java.com/)
[![License](https://img.shields.io/badge/License-MIT-6DB33F?style=flat-square)](LICENSE)

</div>

---

<div align="center">

| | | |
|:---:|:---:|:---:|
| ![](images/01.JPG) | ![](images/02.JPG) | ![](images/03.JPG) |

</div>

---

## Table of Contents

- [Overview](#overview)
- [Branch Naming Scheme](#branch-naming-scheme)
- [AdvantageScope: Robot\_Shaq Asset](#advantagescope-robot_shaq-asset)

---

## Overview

This repository contains all robot code for **Shaquille O'Steel**, Steel Hawks' 2026 competition robot.

---

## Branch Naming Scheme

| Prefix | Purpose |
|--------|---------|
| `main` | Stable, competition-ready code |
| `feat/` | New features under development |
| `bugfix/` | Bug fixes |
| `refactor/` | Code cleanup and restructuring |
| `sandbox/` | Experimental work, not for merge |
| `event/` | Event-specific branches |

---

## AdvantageScope: Robot\_Shaq Asset

This section explains how to install the custom **Robot\_Kirin** 3D model with articulated subsystems in AdvantageScope.

### Step 1 — Open the Assets Folder

1. Open AdvantageScope
2. Navigate to **App → AdvantageScope → Show Assets Folder**
3. A folder will open on your computer — this is the root assets directory

### Step 2 — Drag the Folder In

Drag the entire **`Robot_Shaq`** folder into the AdvantageScope assets folder.

> [!IMPORTANT]
> Drag the **entire folder**, not individual files inside it.

The final structure should look like this:

```
AdvantageScopeAssets/
└── Robot_Shaq/
    ├── model.glb
    ├── config.json
    └── ...
```

### Step 3 — Restart AdvantageScope

After restarting, **Shaq** should appear in the robot selection dropdown.

### Step 4 — Add the NT Fields

With the robot model selected, drag and drop the following fields onto the **Robot** component in AdvantageScope — **in this order**:

```
NT:/AdvantageKit/RealOutputs/Superstructure/ComponentPoses
NT:/AdvantageKit/RealOutputs/Intake/ComponentPoses
```

> [!NOTE]
> Articulation currently only works in simulation.

![AdvantageScope setup](images/image.png)