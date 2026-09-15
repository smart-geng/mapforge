"""Run ORBIT's unmodified core in a separate interpreter, never mapforge's venv.

This is an editor-admission experiment, not a converter or a map acceptance.
Derived display control points do not verify the input CRS. No network map data
is fetched. The GUI's optional post-import road realignment is NOT called.
"""
import argparse
from dataclasses import asdict
import hashlib
from importlib.metadata import distributions
import json
from pathlib import Path
import sys

REVISION = "8bd191af7934d7e7f236229648c5716108ec042f"


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2,
                                    allow_nan=False), encoding="utf-8")


def run(source, output, vendor):
    from orbit_core.models.project import Project
    from orbit_core.importers.opendrive_importer import (
        OpenDriveImporter, ImportOptions, ImportMode)
    from orbit_core.export.opendrive_writer import OpenDriveWriter, ExportOptions
    from orbit_core.utils.coordinate_transform import create_transformer

    source, output, vendor = source.resolve(), output.resolve(), vendor.resolve()
    output.mkdir(parents=True, exist_ok=False)
    bound = {str(source): digest(source)}
    # Freeze downloaded implementation, not just the claimed revision string.
    for path in sorted(vendor.rglob("*")):
        if path.is_file() and (path.suffix in (".py", ".toml") or path.name == "LICENSE"):
            bound[str(path)] = digest(path)
    bound[str(Path(__file__).resolve())] = digest(__file__)
    registration = dict(upstream_revision=REVISION, input_sha256=bound[str(source)],
                        input=str(source), python=sys.version,
                        scope="core-no-edit-roundtrip; NOT GUI drag or map acceptance",
                        import_mode="replace", preserve_geometry=True,
                        output_version="1.8 default; target 1.5 compatibility NOT claimed",
                        gui_realign_called=False, source_crs_verified=False,
                        control_points="derived from input CRS for display only",
                        source_files_modified=False, input_hashes=bound)
    save_json(output / "registration.json", registration)
    save_json(output / "packages.json", sorted(
        ((d.metadata['Name'], d.version) for d in distributions()), key=lambda x:x[0]))

    project = Project()
    project.synthetic_canvas_width = project.synthetic_canvas_height = 6000
    importer = OpenDriveImporter(project, None, 6000, 6000)
    result = importer.import_from_file(str(source), ImportOptions(
        import_mode=ImportMode.REPLACE, auto_create_control_points=True))
    save_json(output / "import-result.json", asdict(result))
    if not result.success:
        raise RuntimeError(result.error_message)
    if not project.imported_geo_reference:
        raise ValueError("Missing input CRS: no default CRS is permitted")

    project.save(output / "imported.orbit")
    stages = []
    for name, current in (("direct", project),
                          ("reopened", Project.load(output / "imported.orbit"))):
        transformer = create_transformer(
            current.control_points, current.transform_method,
            export_proj_string=current.imported_geo_reference)
        if transformer is None:
            raise RuntimeError("ORBIT could not recreate the input projection")
        writer = OpenDriveWriter(current, transformer, options=ExportOptions(
            country_code="cn", geo_reference_string=current.imported_geo_reference,
            offset_x=0.0, offset_y=0.0, carla_compat=False))
        target = output / (name + ".xodr")
        success = writer.write(str(target))
        stages.append(dict(stage=name, write_success=success,
                           reference_warnings=writer.reference_warnings,
                           export_warnings=writer.export_warnings,
                           road_id_map=writer.road_numeric_ids,
                           sha256=digest(target) if success else None))
    changed = [path for path, value in bound.items() if digest(path) != value]
    report = dict(status="WRITTEN_NOT_ACCEPTED", stages=stages,
                  immutable_inputs_changed=changed, map_accepted=False,
                  source_crs_verified=False, gui_tested=False)
    save_json(output / "worker-result.json", report)
    if changed or not all(x['write_success'] for x in stages):
        raise RuntimeError("roundtrip incomplete or original input changed")
    print(json.dumps(report), flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("vendor", type=Path)
    args = parser.parse_args()
    run(args.source, args.output, args.vendor)
