from pathlib import Path

import cantools
from jinja2 import Environment, FileSystemLoader

from esw.can.dbc.parser import prepare_context


def generate_can_header(ctx: Path, dest: Path, files: list[str]) -> None:
    env = Environment(loader=FileSystemLoader(ctx))
    combined_template = env.get_template("templates/dbc_header.hpp.j2")

    for f in files:
        f_path = Path(f)
        try:
            db = cantools.database.load_file(f_path)
            dbc_name = f_path.stem
            combined_context = prepare_context(db, dbc_name)
            rendered_combined = combined_template.render(combined_context)

            if not dest.exists():
                dest.mkdir(parents=True, exist_ok=True)

            combined_header_file = dest / f"{dbc_name}.hpp"
            with open(combined_header_file, "w") as handle:
                handle.write(rendered_combined)

        except Exception as e:
            print(f"Error processing {f}: {e}")
