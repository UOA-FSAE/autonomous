# Autonomous

## About Formula SAE and the Autonomous Competition

Formula SAE (FSAE) is a student design competition organized by SAE International (previously known as the Society of Automotive Engineers). In this competition, student teams from universities around the world design, build, and test small, formula-style race cars. The cars are then judged based on a variety of criteria including design, cost, presentation, and on-track performance in dynamic events.

In recent years, an Autonomous competition has been introduced to the FSAE suite of events. In this competition, teams must develop and integrate autonomous systems into their vehicles. The goal is to have the car navigate around a track without any human intervention, using a combination of onboard sensors, processing power, and control systems to perceive the environment, make decisions, and control the vehicle. This challenge provides an opportunity for students to develop skills in areas such as machine learning, computer vision, sensor fusion, control systems, and robotics, all within a highly competitive and practical context.

We are proud to participate in this challenging and innovative competition. We are committed to pushing the boundaries of autonomous vehicle technology and fostering the next generation of engineers. Through this GitHub repository, you can explore our work and contribute to the advancement of autonomous driving technologies.

## Documentation

For detailed documentation about this project, please visit the **Wiki** tab on this GitHub repository. Here you'll find comprehensive guides, explanations, and resources to help you understand and navigate the project. Don't hesitate to explore the Wiki and utilize it as a valuable resource throughout your development process.

## Dev Environment

### Prerequisites

- GNUmake:
  - Install on Windows using Chocolatey.
  - Install on Mac using Homebrew: `brew install make`.
  - Install on Linux using apt: `apt install make`.

- VS code:
  - [Remote Development Extension Pack](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack)

- Git: [Install Link](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git)

- Docker (Compose and Engine): [Install Link](https://docs.docker.com/engine/install/)

### Local Dev Environment - First Time Setup

1. Clone repo using `git clone https://github.com/UOA-FSAE/autonomous.git`.
2. `cd` into repo.
3. Run `make build` (Use Git Bash if on Windows).
4. In VS Code under the `Ctrl+Shift+P` menu, run `Dev Containers: Rebuild and Reopen in Container` or run `make start`. Or if prompted, click `reopen in container` from popup.

### Local Dev Environment - Normal Usage

**Method 1:**
- Go to Remote Explorer.
- Click icon next to autonomous devcontainer (open folder in container).

**Method 2:**
- In VS Code under the `Ctrl+Shift+P` menu, run `Dev Containers: Rebuild and Reopen in Container` (with repo open in vscode).

### Codespaces Dev Environment

- Go to Remote Explorer.
- Change to GitHub Codespaces tab.
- Click connect to codespace icon next to desired codespace.

### Ros2

Running commands: `ros2 <command>`
