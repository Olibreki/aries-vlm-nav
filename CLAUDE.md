# Project Context: ARIES Lab — VLM/VLA Nav Stack

Context for Claude. Read this before helping with anything in this project. (A copy of this file lives in the repo root as `CLAUDE.md`, so Claude Code reads it automatically.)

## Who I am

- **Olafur (Oli) Breki Gudnason.** M.S. in Autonomy (Embodied AI and Robotics Systems) at Purdue, Aug 2026 – May 2028.
- **Background:** B.Sc. in Electrical and Computer Engineering (University of Iceland), post-bacc in CS, industry ML/automation work at ON Power.
- **Also going on this semester:** the [[ros2-localization-benchmark]] project (separate repo, own scope — EKF/particle filter, don't mix the two), a course team project, and Summer 2027 internship applications.

## Why this project exists

Prep and research track for volunteering in **Prof. Upinder Kaur's ARIES Lab** (Purdue ABE), working with PhD student **Aathman Tharmasanthiran**. His direction is Vision-Language-Action models learning robot skills from human demonstration video. The near-term ask (per the Sep 25, 2026 meeting) is to get credible on the navigation half of the stack: VLM/VLA → 2D pointing → tf2 → Nav2 goal, plus Habitat/BEHAVIOR-1K for benchmarking.

Full plan and reading list: `vlm-nav-stack-week-plan` in the SKYNET vault (`02-Areas/career/research/vlm-nav-stack-week-plan.md`).

**Note:** whether this is *paid/formal* lab work or informal is still unresolved with Purdue ISS (F-1 compliance question) — see `kaur-aries-lab` in the vault. This repo is research/learning work regardless of how that resolves.

## Where I'm starting from

- Already comfortable with ROS 2 Jazzy + Gazebo + Nav2 + TurtleBot3 from `ros2-localization-benchmark` — reuse that sim stack, don't re-derive it.
- **New to:** tf2 in anger (pixel → 3D → map frame deprojection), `nav2_simple_commander`, and VLM pointing APIs (Molmo / Qwen3-VL / Gemma).
- **Dev machine:** same as the localization benchmark — Windows laptop, WSL2 Ubuntu 24.04, integrated graphics only, **no usable local GPU**. This hard-splits the stack; see the week-plan's hardware table. Anything needing a GPU (Habitat-sim beyond Colab, BEHAVIOR-1K/OmniGibson) is read-only/Colab-only this week, not built here.

## Fixed decisions (don't re-litigate unless something is broken)

| Area | Decision |
|---|---|
| Environment | Ubuntu 24.04 in WSL2 on Windows; integrated graphics only |
| ROS | ROS 2 Jazzy, same sim stack as `ros2-localization-benchmark` (Gazebo Harmonic, TurtleBot3) |
| Language | Python (rclpy) — `nav2_simple_commander` and the tf2 buffer API used here are Python-first; C++ stays in the other repo |
| Scope this week | Mini-project 4.1 (pixel → map → Nav2 goal) is the priority deliverable. 4.2 (pointing bake-off) and 4.3 (BDDL reader) may land here too if time allows; 4.4 (Habitat Colab) has no local code |
| VLM calls | Hosted APIs only (Molmo demo, Qwen3-VL API, Gemma via AI Studio) — no local inference, no GPU |
| Build | colcon, `src/` layout, package(s) created with `ros2 pkg create` as needed |

## How I want Claude to help

- **Teach, don't just produce.** I need to be able to explain this to Aathman and Prof. Kaur, not just have it run. Explain the tf2/geometry math, not just the API calls.
- **Small steps, run it myself.** Tell me the next concrete step and how to verify it (a topic to echo, a marker to see in RViz), let me type/run it, unless I ask you to run it directly.
- **WSL2 awareness.** No GPU, integrated graphics. Prefer headless Gazebo + RViz when the GUI is slow. Flag when online ROS advice targets a different distro/sim than Jazzy/Harmonic.
- **Flag scope creep toward the GPU-only pieces** (OmniGibson, local Habitat installs, fine-tuning) — the week plan explicitly rules those out; redirect me to Colab/reading instead if I start down that path.
- **Division of work.** I write/understand the tf2 transform logic myself since I'll need to explain it; Claude Code can handle ROS plumbing (package.xml, launch files, CMake-equivalent Python setup.py), debugging, and boilerplate.

## Portfolio / resume rules

- This is research-track work for a specific lab meeting, not a resume line yet. Don't claim results that aren't from an actual run in this repo.
- If it matures into something resume-worthy, that decision happens in the vault (`kaur-aries-lab` / `resume-strategy`), not here.
