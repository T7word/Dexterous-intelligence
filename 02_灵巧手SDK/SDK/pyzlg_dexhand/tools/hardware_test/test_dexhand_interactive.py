import os
import sys
import yaml
import argparse
from typing import List
from IPython import embed

from pyzlg_dexhand.dexhand_interface import (
    LeftDexHand,
    RightDexHand,
    ControlMode,
    ZCANWrapper,
    FeedbackMode,
    JointCommand,
)


def initialize_hands(hand_names: List[str]) -> dict:
    """Initialize specified hands and return a dictionary of instances"""
    hands_dict = {}
    zcan = None

    if len(hand_names) > 1:
        # Create shared ZCAN instance if using both hands
        zcan = ZCANWrapper()
        if not zcan.open():
            print("Failed to open shared ZCAN instance")
            return {}

    for name in hand_names:
        if name == "left":
            hand = LeftDexHand(zcan)
        elif name == "right":
            hand = RightDexHand(zcan)
        else:
            raise ValueError(f"Unknown hand name: {name}")

        if not hand.init():
            print(f"Failed to initialize {name} hand")
            continue

        # Check firmware version
        versions = hand.get_firmware_versions()
        if versions:
            # Print firmware versions for this hand
            print(f"\n{name.upper()} HAND FIRMWARE VERSIONS:")
            for joint, version in versions.items():
                if version is not None:
                    print(f"  {joint}: {version}")
            
            # Get unique versions
            unique_versions = set(v for v in versions.values() if v is not None)
            if len(unique_versions) > 1:
                print(f"ERROR: {name} hand has mismatched firmware versions: {unique_versions}")
                print("Different versions across boards can cause unpredictable behavior.")
                print("It is recommended to update all boards to the same version.")
                # Don't exit - this is interactive mode, just warn
            elif len(unique_versions) == 0:
                print(f"WARNING: Could not read firmware versions for {name} hand")
            else:
                print(f"{name} hand unified firmware version: {list(unique_versions)[0]}")

        print(f"Initialized {name} hand")
        hands_dict[name] = hand

    return hands_dict


def main():
    parser = argparse.ArgumentParser(description="Interactive dexterous hand control")
    parser.add_argument(
        "--hands",
        nargs="+",
        choices=["left", "right"],
        default=["left"],
        help="Which hands to initialize (default: left)",
    )
    args = parser.parse_args()
    # Initialize hands
    hands = initialize_hands(args.hands)

    if not hands:
        print("No hands initialized successfully. Exiting.")
        return

    # Create globals dict for IPython
    globals_dict = {
        "ControlMode": ControlMode,
        "FeedbackMode": FeedbackMode,
        "JointCommand": JointCommand,
        "hands": hands,
    }
    for i, hand in enumerate(hands):
        globals_dict[f"{args.hands[i]}_hand"] = hands[hand]

    print("Hands initialized. Entering IPython shell...")
    print("Available globals:")
    print(f"  hands: List of initialized hand instances")
    print(f"  JointCommand: Class for detailed joint control (position, current, velocity)")
    print(f"  ControlMode: Enum of available control modes")
    print(f"  FeedbackMode: Enum of available feedback modes")
    for name in args.hands:
        print(f"  {name}_hand: {name.title()} hand instance")

    print("\nAvailable Joints:")
    print("  Thumb:")
    print("    th_rot  - Thumb rotation (0-150 degrees)")
    print("    th_mcp  - Thumb metacarpophalangeal flexion (0-90 degrees)")
    print("    th_dip  - Thumb distal joints coupled flexion (0-90 degrees)")
    print("  Fingers:")
    print("    ff_spr  - Four-finger spread/abduction (0-30 degrees)")
    print("    ff_mcp  - Index finger metacarpophalangeal flexion (0-90 degrees)")
    print("    ff_dip  - Index finger distal joints flexion (0-90 degrees)")
    print("    mf_mcp  - Middle finger metacarpophalangeal flexion (0-90 degrees)")
    print("    mf_dip  - Middle finger distal joints flexion (0-90 degrees)")
    print("    rf_mcp  - Ring finger metacarpophalangeal flexion (0-90 degrees)")
    print("    rf_dip  - Ring finger distal joints flexion (0-90 degrees)")
    print("    lf_mcp  - Little finger metacarpophalangeal flexion (0-90 degrees)")
    print("    lf_dip  - Little finger distal joints flexion (0-90 degrees)")

    print("\nExample Commands:")
    print("  Basic position control (broadcast mode used by default):")
    print(
        f"    {args.hands[i]}_hand.move_joints(th_rot=30, th_mcp=45)        # Move thumb"
    )
    print(
        f"    {args.hands[i]}_hand.move_joints(ff_spr=20)                   # Spread fingers"
    )
    
    print("\n  Advanced control with JointCommand:")
    print(
        f"    {args.hands[i]}_hand.move_joints(ff_mcp=JointCommand(position=45, current=500, velocity=100))  # Position with current and velocity limits"
    )
    print(
        f"    {args.hands[i]}_hand.move_joints(th_mcp=JointCommand(position=30, current=300))                # Position with current limit only"
    )
    
    print("\n  Using different control modes (broadcast mode used by default):")
    print(
        f"    {args.hands[i]}_hand.move_joints(ff_mcp=90, ff_dip=90, control_mode=ControlMode.PROTECT_HALL_POSITION)       # Protected position control"
    )
    print(
        f"    {args.hands[i]}_hand.move_joints(ff_mcp=JointCommand(position=45, current=300), control_mode=ControlMode.IMPEDANCE_GRASP)  # Impedance grasp mode"
    )
    print(
        f"    {args.hands[i]}_hand.move_joints(ff_mcp=JointCommand(position=0, current=200), control_mode=ControlMode.MIT_TORQUE)         # MIT torque mode"
    )
    print(
        f"    {args.hands[i]}_hand.move_joints(th_rot=45, use_broadcast=False)  # Use per-board commands instead of broadcast mode"
    )

    print("\n  Other commands:")
    print(
        f"    {args.hands[i]}_hand.reset_joints()           # Move all joints to zero position (uses broadcast mode by default)"
    )
    print(
        f"    {args.hands[i]}_hand.reset_joints(use_broadcast=False)  # Reset all joints using per-board commands (slower)"
    )
    print(
        f"    {args.hands[i]}_hand.get_feedback()           # Get current joint and touch feedback"
    )
    print(
        f"    {args.hands[i]}_hand.clear_errors(use_broadcast=False)  # Clear any error states (use_broadcast=False to avoid a known bug)"
    )

    embed(user_ns=globals_dict)

    # Cleanup
    print("Closing hands...")
    for name, hand in hands.items():
        print(f"Closing {name} hand...")
        hand.close()
    print("Exiting")


if __name__ == "__main__":
    main()
