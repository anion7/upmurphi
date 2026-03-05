#!/usr/bin/env python3
"""
UPMurphi Plan Visualizer

Creates a Gantt-chart visualization of plan execution and an animated
simulation replay window using matplotlib.

Usage:
  python3 plan_visualizer.py <plan_file> [--animate]
"""

import os
import sys
import time
import argparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from upmurphi_plan_parser import parse_plan_file, Plan

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation
import numpy as np


COLORS = [
    '#4285F4', '#EA4335', '#FBBC04', '#34A853',
    '#FF6D01', '#46BDC6', '#7B61FF', '#E91E63',
    '#00BCD4', '#8BC34A', '#FF5722', '#607D8B',
]


def get_action_color(name: str, palette: dict) -> str:
    if name not in palette:
        palette[name] = COLORS[len(palette) % len(COLORS)]
    return palette[name]


def plot_gantt(plan: Plan, save_path: str = None):
    """Draw a static Gantt chart of the plan."""
    if not plan.actions:
        print("No actions to plot.")
        return

    fig, ax = plt.subplots(figsize=(14, max(4, len(plan.actions) * 0.6 + 1.5)))
    palette = {}

    for i, action in enumerate(plan.actions):
        color = get_action_color(action.action_name, palette)
        bar = ax.barh(
            i, action.duration, left=action.timestamp,
            height=0.6, color=color, edgecolor='white', linewidth=0.8,
            alpha=0.85,
        )
        label = f"{action.action_name}({', '.join(action.parameters)})"
        cx = action.timestamp + action.duration / 2
        ax.text(cx, i, label, ha='center', va='center',
                fontsize=8, color='white', fontweight='bold',
                clip_on=True)

    ax.set_yticks(range(len(plan.actions)))
    ax.set_yticklabels(
        [f"A{i+1}" for i in range(len(plan.actions))],
        fontsize=9,
    )
    ax.invert_yaxis()
    ax.set_xlabel('Time (s)', fontsize=11)
    ax.set_title(
        f'UPMurphi Plan #{plan.plan_number:05d}  —  '
        f'{plan.num_actions} actions, duration={plan.total_duration:.1f}s',
        fontsize=13, fontweight='bold', pad=12,
    )
    ax.grid(axis='x', alpha=0.3, linestyle='--')
    ax.set_axisbelow(True)

    handles = [
        mpatches.Patch(color=c, label=n) for n, c in palette.items()
    ]
    ax.legend(
        handles=handles, loc='upper right', fontsize=8,
        framealpha=0.9, title='Actions', title_fontsize=9,
    )

    plt.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Gantt chart saved to {save_path}")
    return fig, ax


def animate_dispatch(plan: Plan, save_path: str = None):
    """Animated plan dispatch visualization."""
    if not plan.actions:
        print("No actions to animate.")
        return

    palette = {}
    for a in plan.actions:
        get_action_color(a.action_name, palette)

    total_dur = max(a.end_time for a in plan.actions)
    n = len(plan.actions)

    fig, (ax_gantt, ax_status) = plt.subplots(
        2, 1, figsize=(14, max(6, n * 0.5 + 4)),
        gridspec_kw={'height_ratios': [3, 1]},
    )
    fig.suptitle(
        f'UPMurphi Plan Dispatch Simulation  —  Plan #{plan.plan_number:05d}',
        fontsize=13, fontweight='bold',
    )

    ax_gantt.set_xlim(-0.5, total_dur * 1.05)
    ax_gantt.set_ylim(-0.5, n - 0.5)
    ax_gantt.set_yticks(range(n))
    ax_gantt.set_yticklabels([f"A{i+1}" for i in range(n)], fontsize=9)
    ax_gantt.invert_yaxis()
    ax_gantt.set_xlabel('Time (s)', fontsize=10)
    ax_gantt.grid(axis='x', alpha=0.3, linestyle='--')
    ax_gantt.set_axisbelow(True)

    bg_bars = []
    for i, action in enumerate(plan.actions):
        color = get_action_color(action.action_name, palette)
        bar = ax_gantt.barh(
            i, action.duration, left=action.timestamp,
            height=0.6, color=color, alpha=0.15,
            edgecolor=color, linewidth=0.8, linestyle='--',
        )
        bg_bars.append(bar)
        label = f"{action.action_name}({', '.join(action.parameters)})"
        ax_gantt.text(
            action.timestamp + action.duration / 2, i, label,
            ha='center', va='center', fontsize=7, color='gray',
        )

    progress_bars = []
    for i in range(n):
        bar = ax_gantt.barh(
            i, 0, left=plan.actions[i].timestamp,
            height=0.6, color=get_action_color(plan.actions[i].action_name, palette),
            alpha=0.85, edgecolor='white', linewidth=0.8,
        )
        progress_bars.append(bar)

    time_line = ax_gantt.axvline(x=0, color='red', linewidth=1.5, linestyle='-', alpha=0.7)

    ax_status.set_xlim(0, 1)
    ax_status.set_ylim(0, 1)
    ax_status.axis('off')
    status_text = ax_status.text(
        0.5, 0.7, '', ha='center', va='center', fontsize=14, fontweight='bold',
    )
    detail_text = ax_status.text(
        0.5, 0.3, '', ha='center', va='center', fontsize=11, color='#555',
    )

    num_frames = 200
    times = np.linspace(0, total_dur, num_frames)

    def update(frame):
        t = times[frame]
        time_line.set_xdata([t, t])

        for i, action in enumerate(plan.actions):
            if t >= action.timestamp:
                elapsed = min(t - action.timestamp, action.duration)
                progress_bars[i][0].set_width(elapsed)
            else:
                progress_bars[i][0].set_width(0)

        current_action = None
        for i, action in enumerate(plan.actions):
            if action.timestamp <= t < action.end_time:
                current_action = (i, action)

        if current_action:
            i, action = current_action
            pct = (t - action.timestamp) / action.duration * 100
            status_text.set_text(f'▶ Executing: {action.action_name}')
            status_text.set_color(get_action_color(action.action_name, palette))
            detail_text.set_text(
                f'Parameters: {", ".join(action.parameters)}  |  '
                f'Progress: {pct:.0f}%  |  Time: {t:.1f}s'
            )
        elif t >= total_dur:
            status_text.set_text('✓ Plan Dispatch Complete')
            status_text.set_color('#34A853')
            detail_text.set_text(
                f'{plan.num_actions} actions executed  |  '
                f'Total duration: {plan.total_duration:.1f}s'
            )
        else:
            status_text.set_text(f'Waiting... (t={t:.1f}s)')
            status_text.set_color('#999')
            detail_text.set_text('')

        return [time_line] + progress_bars + [status_text, detail_text]

    anim = FuncAnimation(
        fig, update, frames=num_frames,
        interval=50, blit=False, repeat=False,
    )

    plt.tight_layout()
    if save_path:
        anim.save(save_path, writer='pillow', fps=20, dpi=100)
        print(f"Animation saved to {save_path}")

    return fig, anim


def main():
    parser = argparse.ArgumentParser(description="UPMurphi Plan Visualizer")
    parser.add_argument("plan_file", help="UPMurphi plan output file")
    parser.add_argument("--animate", action="store_true", help="Show animated dispatch")
    parser.add_argument("--save-gantt", type=str, default=None, help="Save Gantt chart")
    parser.add_argument("--save-anim", type=str, default=None, help="Save animation (GIF)")
    parser.add_argument("--plan-index", type=int, default=0)
    args = parser.parse_args()

    plans = parse_plan_file(args.plan_file)
    if not plans:
        print("No plans found.")
        sys.exit(1)

    plan = plans[args.plan_index]
    print(f"Plan #{plan.plan_number}: {plan.num_actions} actions, "
          f"duration={plan.total_duration:.1f}s")

    if args.save_gantt:
        plot_gantt(plan, save_path=args.save_gantt)

    if args.animate or args.save_anim:
        fig, anim = animate_dispatch(plan, save_path=args.save_anim)
        if not args.save_anim:
            plt.show()
    elif not args.save_gantt:
        fig, ax = plot_gantt(plan)
        plt.show()


if __name__ == "__main__":
    main()
