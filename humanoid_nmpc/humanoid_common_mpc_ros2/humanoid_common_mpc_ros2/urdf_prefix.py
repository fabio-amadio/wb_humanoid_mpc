import argparse
import sys
import xml.etree.ElementTree as ET


def prefix_urdf_links(urdf_path: str, link_prefix: str) -> str:
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    for link in root.findall("link"):
        name = link.get("name")
        if name:
            link.set("name", link_prefix + name)

    for joint in root.findall("joint"):
        parent = joint.find("parent")
        if parent is not None:
            parent_link = parent.get("link")
            if parent_link:
                parent.set("link", link_prefix + parent_link)

        child = joint.find("child")
        if child is not None:
            child_link = child.get("link")
            if child_link:
                child.set("link", link_prefix + child_link)

    for gazebo in root.findall("gazebo"):
        reference = gazebo.get("reference")
        if reference:
            gazebo.set("reference", link_prefix + reference)

    return ET.tostring(root, encoding="unicode")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Prefix URDF link names and link references."
    )
    parser.add_argument("urdf_path")
    parser.add_argument("link_prefix")
    args = parser.parse_args()

    sys.stdout.write(prefix_urdf_links(args.urdf_path, args.link_prefix))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
