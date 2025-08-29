"""Generate the code reference pages and navigation."""

from pathlib import Path
import mkdocs_gen_files

# Package structure mapping
package_paths = {
    "adg_ros2_utils": "adg_ros2_utils/adg_ros2_utils",
    "robot_arm_bringup": "robot_arm/robot_arm_bringup/robot_arm_bringup",
    "robot_arm_client": "robot_arm/robot_arm_client/robot_arm_client",
    "robot_arm_interface": "robot_arm/robot_arm_interface/robot_arm_interface",
    "robot_tasks": "robot_tasks/robot_tasks",
    "franka_rim": "robot_arm/franka_rim/franka_rim",
    "isaac_sim_ros": "robot_arm/isaac_sim_ros/isaac_sim_ros",
    "teleop": "teleop/teleop/teleop",
    "inverse3_ros2": "teleop/inverse3_ros2/inverse3_ros2",
    "network_sim": "teleop/network_sim/network_sim",
}

nav_lines = []

for package_name, package_path in package_paths.items():
    package_dir = Path(package_path)

    # Check if the package directory exists
    if not package_dir.exists():
        print(f"Warning: Package directory {package_dir} does not exist")
        continue

    # Look for Python modules in the package
    python_files = list(package_dir.rglob("*.py"))
    if not python_files:
        print(f"Warning: No Python files found in {package_dir}")
        continue

    # Add package section header
    nav_lines.append(f"- {package_name.replace('_', ' ').title()}")

    for path in sorted(python_files):
        # Skip __pycache__ directories
        if "__pycache__" in str(path):
            continue

        # Get the module path relative to the package directory
        try:
            relative_path = path.relative_to(package_dir)
            module_parts = list(relative_path.with_suffix("").parts)

            # Skip __init__ and __main__ files for now
            if module_parts[-1] in ["__init__", "__main__"]:
                continue

            # Create the module identifier for mkdocstrings
            module_identifier = f"{package_name}.{'.'.join(module_parts)}"

            # Create the documentation path
            doc_path = Path("reference") / package_name / f"{'.'.join(module_parts)}.md"

            nav_lines.append(
                f"  - {module_parts[-1].replace('_', ' ').title()}: reference/{package_name}/{'.'.join(module_parts)}.md"
            )

            with mkdocs_gen_files.open(doc_path, "w") as fd:
                print(f"# {module_identifier}", file=fd)
                print(f"::: {module_identifier}", file=fd)

            mkdocs_gen_files.set_edit_path(doc_path, path)

        except Exception as e:
            print(f"Error processing {path}: {e}")
            continue

# Create a simple index for the reference section
with mkdocs_gen_files.open("reference/index.md", "w") as fd:
    print("# API Reference", file=fd)
    print("", file=fd)
    print("This section contains the API documentation automatically generated from docstrings.", file=fd)
    print("", file=fd)
    print("## Available Packages", file=fd)
    print("", file=fd)
    for package_name in package_paths.keys():
        package_dir = Path(package_paths[package_name])
        if package_dir.exists():
            print(f"- [{package_name.replace('_', ' ').title()}]({package_name}/)", file=fd)

# Write the navigation file
with mkdocs_gen_files.open("reference/SUMMARY.md", "w") as nav_file:
    nav_file.write("# API Reference\n\n")
    nav_file.write("This section contains the API documentation automatically generated from docstrings.\n\n")
    for line in nav_lines:
        nav_file.write(line + "\n")
