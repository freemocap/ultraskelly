"""The controllers' settings, tuned from the dashboard while the bot runs and saved for its next start.

Each group is one controller's pydantic params, found by its place in the launch configuration
(GROUPS). The dashboard's Tune window draws its controls from the group's JSON schema (titles,
descriptions, bounds and defaults), so a controller's new or renamed settings appear without a
change here. Not tuned: servo and motor channels (any setting named `channel` or `*_channel`),
settings excluded from dumps, a group's `hidden` settings, and any marked
`json_schema_extra={"tuning": False}`. A setting marked `{"tuning": "start"}`, or in a group's
`at_start`, is read only as the bot starts: changed here, it takes effect at the next start. A
setting's unit is its `json_schema_extra={"unit": ...}`, else what its description or its name
says (unit_of). Nested params are tuned too, setting by setting.

A change is checked against the group's whole model, its cross-field rules included, and applied
all or nothing by setting attributes on the running params objects, which the controllers read
every control step. The hard limits (ActuatorLimits) still clamp every command sent.

A setting's default is the launch configuration's own value (launch_config), before any saved
tuning: what Reset returns to, and what the Tune window marks a setting as differing from. Save
keeps the settings that differ from their defaults in TUNING_FILE,

    {"saved_at": "2026-11-07T21:14:03", "groups": {"aim": {"deadband": 0.2}, "waist": {"enabled": false}}}

replaced whole (written beside it, flushed, then renamed over it), so a power cut leaves the old
file or the new one. As the bot starts, launch_config merges it over its own values (load_tuning).
A saved setting that no longer fits, renamed or out of bounds since a controller changed, is
skipped and reported, and the rest still apply; nothing in the file can keep the bot from starting.
"""
import asyncio
import json
import logging
import os
import re
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Iterator

from pydantic import BaseModel, Field, ValidationError
from pydantic.fields import FieldInfo
from pydantic_core import to_jsonable_python

from ultraskelly.core.bot.base_abcs import NodeParams

logger = logging.getLogger(__name__)

TUNING_FILE = Path.home() / ".config" / "ultraskelly" / "tuning.json"  # The bot's user's, on the robot


@dataclass(frozen=True)
class Group:
    """One controller's settings: `path` names its params object in the launch configuration."""

    key: str  # Its name in the API and in the saved file
    title: str
    path: tuple[str, ...]
    hidden: frozenset[str] = frozenset()  # Its settings not tuned here, besides channels
    at_start: frozenset[str] = frozenset()  # Its settings read only as the bot starts


GROUPS = (
    Group("aim", "Head aim", ("orientation", "aim")),
    Group("roll", "Roll", ("orientation", "roll")),
    Group("waist", "Waist", ("orientation", "waist"), at_start=frozenset({"slow_decay"})),  # Set on the motor as it opens
    Group("jaw", "Jaw", ("mouth",), hidden=frozenset({"startup_test"})),  # A test that runs only at start
)

# What a description says a setting's unit is, checked in order: the first that matches wins.
UNITS_SAID = (
    (re.compile(r"servo degrees per second", re.I), "servo°/s"),
    (re.compile(r"degrees per second squared", re.I), "°/s²"),
    (re.compile(r"per second squared", re.I), "/s²"),
    (re.compile(r"degrees per second|deg/s", re.I), "°/s"),
    (re.compile(r"pixels per degree", re.I), "px/°"),
    (re.compile(r"^degrees\b.*\bper\b.*\bservo degree", re.I), "°/servo°"),
    (re.compile(r"^servo degrees", re.I), "servo°"),
    (re.compile(r"^pixels", re.I), "px"),
    (re.compile(r"^seconds", re.I), "s"),
    (re.compile(r"per degree", re.I), "/°"),
    (re.compile(r"per second", re.I), "/s"),
)
# Else what its name says.
UNITS_NAMED = (
    (re.compile(r"_(seconds|duration|timeout|lag|latency|after|gap|settle)$"), "s"),
    (re.compile(r"_(angle|degrees)$"), "°"),
)


def unit_of(name: str, description: str) -> str | None:
    """A setting's unit as its description says it ("Servo degrees of play ...", "... degrees per second"), else as
    its name does (`*_seconds`, `*_angle`), else None: shares, throttles, counts and anything not said."""
    for pattern, unit in UNITS_SAID:
        if pattern.search(description):
            return unit
    for pattern, unit in UNITS_NAMED:
        if pattern.search(name):
            return unit
    return None


class TuningParams(NodeParams):
    """The tuning the bot started with (load_tuning), for the dashboard's tuning routes."""

    file: str | None = Field(default=None, description="Where the tuning is saved; None: tuned live only, never saved")
    defaults: dict[str, dict] = Field(
        default_factory=dict, description="Each group's settings as the launch configuration set them, before the saved tuning",
    )
    saved: dict[str, dict] = Field(default_factory=dict, description="The saved settings applied at start, by group")
    saved_at: str | None = Field(default=None, description="When they were saved, local time")
    problems: list[str] = Field(default_factory=list, description="Saved settings that were not applied, and why")


class TuningRefused(ValueError):
    """A change that does not fit: nothing was changed. `status` is the HTTP status that answers it."""

    def __init__(self, reason: str, status: int = 400) -> None:
        super().__init__(reason)
        self.status = status


# --- The settings of a params model ---------------------------------------------------------------------

def _extra(info: FieldInfo) -> dict:
    return info.json_schema_extra if isinstance(info.json_schema_extra, dict) else {}


def _hidden(group: Group, name: str, info: FieldInfo, top: bool) -> bool:
    return (name == "channel" or name.endswith("_channel") or info.exclude is True
            or _extra(info).get("tuning") is False or (top and name in group.hidden))


def _at_start(group: Group, name: str, info: FieldInfo, top: bool) -> bool:
    return _extra(info).get("tuning") == "start" or (top and name in group.at_start)


def _inner(info: FieldInfo) -> type[BaseModel] | None:
    """The params model a setting holds, when it holds settings of its own."""
    kind = info.annotation
    return kind if isinstance(kind, type) and issubclass(kind, BaseModel) else None


def _tuned(group: Group, model: type[BaseModel], top: bool = True) -> Iterator[tuple[str, FieldInfo, type[BaseModel] | None]]:
    """(name, field, its own params model or None) for each setting of `model` tuned here, in the model's order."""
    for name, info in model.model_fields.items():
        if not _hidden(group, name, info, top):
            yield name, info, _inner(info)


def values_of(group: Group, params: BaseModel, top: bool = True) -> dict:
    """The tuned settings of `params` as JSON values, nested params as objects."""
    return {
        name: values_of(group, getattr(params, name), False) if inner is not None else to_jsonable_python(getattr(params, name))
        for name, info, inner in _tuned(group, type(params), top)
    }


def _leaves(group: Group, model: type[BaseModel], settings: dict, top: bool = True, prefix: str = "") -> dict[str, Any]:
    """{"inner": {"gain": 1}} as {"inner.gain": 1}, for the tuned settings `settings` holds."""
    leaves: dict[str, Any] = {}
    for name, info, inner in _tuned(group, model, top):
        if name not in settings:
            continue
        if inner is not None and isinstance(settings[name], dict):
            leaves.update(_leaves(group, inner, settings[name], False, f"{prefix}{name}."))
        else:
            leaves[prefix + name] = settings[name]
    return leaves


def _flat(settings: dict, prefix: str = "") -> dict[str, Any]:
    """Any nested objects as dotted names, for a saved file whose names may no longer exist."""
    flat: dict[str, Any] = {}
    for name, value in settings.items():
        if isinstance(value, dict):
            flat.update(_flat(value, f"{prefix}{name}."))
        else:
            flat[f"{prefix}{name}"] = value
    return flat


def _nest(leaves: dict[str, Any]) -> dict:
    """{"inner.gain": 1} as {"inner": {"gain": 1}}."""
    nested: dict = {}
    for name, value in leaves.items():
        *parents, last = name.split(".")
        place = nested
        for parent in parents:
            place = place.setdefault(parent, {})
        place[last] = value
    return nested


def _merged(base: dict, changes: dict) -> dict:
    merged = dict(base)
    for name, value in changes.items():
        merged[name] = _merged(merged[name], value) if isinstance(value, dict) and isinstance(merged.get(name), dict) else value
    return merged


def _resolved(prop: dict, definitions: dict) -> dict:
    """A property with its $ref (an enum's, say) read from the schema's definitions, less the definition's title,
    which names its class rather than the setting."""
    reference = prop.get("$ref")
    if reference is None:
        return dict(prop)
    definition = definitions.get(reference.rsplit("/", 1)[-1], {})
    return {**{key: value for key, value in definition.items() if key != "title"},
            **{key: value for key, value in prop.items() if key != "$ref"}}


def schema_of(group: Group, model: type[BaseModel], defaults: dict, top: bool = True) -> dict:
    """`model`'s JSON schema for the Tune window: its tuned settings only, nested params inline, each `default` the
    launch configuration's value (what Reset returns to), each with its `unit` when one is known, and "tuning":
    "start" on those read only as the bot starts."""
    full = model.model_json_schema()
    definitions = full.get("$defs", {})
    properties = {}
    for name, info, inner in _tuned(group, model, top):
        raw = full["properties"][name]
        if inner is not None:
            nested = schema_of(group, inner, defaults.get(name) or {}, False)
            properties[name] = {**({"title": raw["title"]} if "title" in raw else {}),
                                "description": raw.get("description") or nested["description"],
                                "type": "object", "properties": nested["properties"]}
            continue
        prop = _resolved(raw, definitions)
        if name in defaults:
            prop["default"] = defaults[name]
        if "unit" not in prop and prop.get("type") != "boolean" and "enum" not in prop:
            unit = unit_of(name, prop.get("description", ""))
            if unit:
                prop["unit"] = unit
        if _at_start(group, name, info, top):
            prop["tuning"] = "start"
        properties[name] = prop
    return {"title": full.get("title", model.__name__), "description": full.get("description", ""), "type": "object",
            "properties": properties}


def _reasons(error: ValidationError, where: str) -> str:
    return "; ".join(f"{'.'.join([where, *map(str, problem['loc'])])}: {problem['msg']}" for problem in error.errors())


def _checked(group: Group, model: type[BaseModel], changes: object, where: str, top: bool = True) -> None:
    """Refuse a change that names a setting the model lacks, as extra="forbid" would, or one not tuned here."""
    if not isinstance(changes, dict) or not changes:
        raise TuningRefused(f'{where}: send its settings as an object, such as {{"name": value}}')
    for name, value in changes.items():
        info = model.model_fields.get(name)
        if info is None:
            raise TuningRefused(f"{where}.{name}: no such setting (extra inputs are not permitted)")
        if _hidden(group, name, info, top):
            raise TuningRefused(f"{where}.{name}: not tuned here")
        inner = _inner(info)
        if inner is not None:
            _checked(group, inner, value, f"{where}.{name}", False)


def validated(group: Group, params: BaseModel, changes: object) -> BaseModel:
    """`params` with `changes` ({setting: value}, nested params as objects), checked by its model as a whole, from
    JSON and strictly: numbers as numbers, true or false for a switch. Raises TuningRefused with the reasons."""
    _checked(group, type(params), changes, group.key)
    merged = _merged(params.model_dump(mode="json"), changes)
    try:
        return type(params).model_validate_json(json.dumps(merged), strict=True)
    except ValidationError as error:
        raise TuningRefused(_reasons(error, group.key)) from None


def _assign(group: Group, live: BaseModel, new: BaseModel, top: bool = True, prefix: str = "") -> list[str]:
    """Set each tuned setting of `live` that `new` changes, in place, as the controllers hold `live`; the names set."""
    changed = []
    for name, info, inner in _tuned(group, type(live), top):
        value = getattr(new, name)
        if inner is not None and isinstance(getattr(live, name), BaseModel):
            changed += _assign(group, getattr(live, name), value, False, f"{prefix}{name}.")
        elif value != getattr(live, name):
            setattr(live, name, value)
            changed.append(prefix + name)
    return changed


def _find(root: object, path: tuple[str, ...]) -> BaseModel | None:
    for name in path:
        root = getattr(root, name, None)
    return root if isinstance(root, BaseModel) else None


# --- Saved tuning ---------------------------------------------------------------------------------------

def write_whole(path: Path, text: str) -> None:
    """Replace `path` with `text`, so that a power cut leaves the old file or the new one, never a part of either."""
    path.parent.mkdir(parents=True, exist_ok=True)
    draft = path.with_name(f".{path.name}.new")
    with open(draft, "w", encoding="utf-8") as file:
        file.write(text)
        file.flush()
        os.fsync(file.fileno())
    os.replace(draft, path)
    if hasattr(os, "O_DIRECTORY"):  # The rename itself reaches the card only once the directory is flushed
        directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)


def load_tuning(root: object, path: Path, groups: tuple[Group, ...] = GROUPS) -> TuningParams:
    """Merge the tuning saved in `path` over the settings in `root` (the launch configuration), in place, and say what
    was merged: the settings before it (the defaults), what applied and what did not. A missing file is no tuning.
    An unreadable file, or a setting that no longer fits, is reported and skipped."""
    found = [(group, _find(root, group.path)) for group in groups]
    found = [(group, params) for group, params in found if params is not None]
    started = TuningParams(file=str(path), defaults={group.key: values_of(group, params) for group, params in found})
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        return started
    except (OSError, ValueError) as error:
        document = None
        started.problems.append(f"{path} could not be read, so none of it applies: {error}")
    if document is not None and not (isinstance(document, dict) and isinstance(document.get("groups"), dict)):
        started.problems.append(f"{path} holds no groups of settings, so none of it applies")
    elif document is not None:
        saved_at = document.get("saved_at")
        started.saved_at = saved_at if isinstance(saved_at, str) else None
        by_key = {group.key: (group, params) for group, params in found}
        for key, settings in document["groups"].items():
            if key not in by_key or not isinstance(settings, dict):
                started.problems.append(f"{key}: no such group of settings; skipped")
                continue
            group, params = by_key[key]
            kept: dict[str, Any] = {}
            for name, value in _flat(settings).items():  # One at a time: what still fits applies
                try:
                    validated(group, params, _nest({**kept, name: value}))
                except TuningRefused as refused:
                    started.problems.append(f"{refused}; skipped")
                    continue
                kept[name] = value
            if kept:
                _assign(group, params, validated(group, params, _nest(kept)))
                started.saved[key] = _nest(kept)
    for problem in started.problems:
        logger.warning(f"Saved tuning: {problem}")
    if started.saved:
        logger.info(f"Saved tuning from {path} applies: {json.dumps(started.saved)}")
    return started


# --- The store the routes use -----------------------------------------------------------------------------

class Tuning:
    """The controllers' settings, tuned live from the dashboard and saved for the bot's next start.

    `root` holds the running params objects at each group's path: the launch configuration, whose objects the
    controllers were given. `started` is what load_tuning merged at start; without it the settings as they are now
    are the defaults and nothing is saved. `on_change` is told which groups changed, after they did, on the event
    loop, between control steps."""

    def __init__(
        self, root: object, started: TuningParams | None = None, *, groups: tuple[Group, ...] = GROUPS,
        on_change: Callable[[set[str]], None] | None = None,
    ) -> None:
        started = started or TuningParams()
        self.root = root
        self.groups = tuple(group for group in groups if _find(root, group.path) is not None)
        self.by_key = {group.key: group for group in self.groups}
        self.defaults = {group.key: started.defaults.get(group.key) or values_of(group, self.params(group))
                         for group in self.groups}
        self.schemas = {group.key: schema_of(group, type(self.params(group)), self.defaults[group.key])
                        for group in self.groups}  # Made once: a schema costs milliseconds of the control loop's time
        self.file = Path(started.file) if started.file else None
        self.saved = dict(started.saved)
        self.saved_at = started.saved_at
        self.problems = list(started.problems)
        self.on_change = on_change
        self.saving = asyncio.Lock()  # One save at a time: two screens' Saves would write the same draft

    def params(self, group: Group) -> BaseModel:
        return _find(self.root, group.path)

    def _group(self, key: object) -> Group:
        if key not in self.by_key:
            raise TuningRefused(f"{key}: no such group of settings; these are tuned: {', '.join(self.by_key)}")
        return self.by_key[key]

    def state(self) -> dict:
        """Each group's schema and settings, which differ from their defaults and which from what the next start
        would apply; the file, when it was saved, and saved settings that were not applied."""
        groups = []
        for group in self.groups:
            params = self.params(group)
            model, values = type(params), values_of(group, params)
            now, defaults = _leaves(group, model, values), _leaves(group, model, self.defaults[group.key])
            kept = {**defaults, **_leaves(group, model, self.saved.get(group.key, {}))}
            groups.append({
                "key": group.key, "title": group.title, "schema": self.schemas[group.key], "values": values,
                "changed": [name for name, value in now.items() if value != defaults.get(name)],
                "unsaved": [name for name, value in now.items() if value != kept.get(name)],
            })
        return {"groups": groups, "file": None if self.file is None else str(self.file), "saved_at": self.saved_at,
                "problems": list(self.problems)}

    def set(self, changes: object) -> list[str]:
        """Apply {group: {setting: value}}, nested params as objects, each group checked by its model: all of it,
        or none of it with TuningRefused. The settings changed, as "group.setting"."""
        if not isinstance(changes, dict) or not changes:
            raise TuningRefused('Send the changes as {"group": {"setting": value}}')
        staged = [(group, self.params(group), validated(group, self.params(group), settings))
                  for group, settings in ((self._group(key), settings) for key, settings in changes.items())]
        changed = [f"{group.key}.{name}" for group, live, new in staged for name in _assign(group, live, new)]
        if changed:
            logger.info(f"Tuned live: {', '.join(changed)}")
            if self.on_change is not None:
                self.on_change({name.split(".")[0] for name in changed})
        return changed

    def reset(self, group: str | None = None, setting: str | None = None) -> list[str]:
        """Back to the defaults, live: every group's, one group's, or one setting's (dotted for a nested one)."""
        if group is None:
            if setting is not None:
                raise TuningRefused("Name the group of the setting to reset")
            return self.set({found.key: self.defaults[found.key] for found in self.groups})
        found = self._group(group)
        if setting is None:
            return self.set({found.key: self.defaults[found.key]})
        defaults = _leaves(found, type(self.params(found)), self.defaults[found.key])
        if setting not in defaults:
            raise TuningRefused(f"{group}.{setting}: no such setting tuned here")
        return self.set({found.key: _nest({setting: defaults[setting]})})

    def to_save(self) -> dict:
        """The settings that differ from their defaults, by group."""
        groups = {}
        for group in self.groups:
            params = self.params(group)
            model = type(params)
            defaults = _leaves(group, model, self.defaults[group.key])
            differ = {name: value for name, value in _leaves(group, model, values_of(group, params)).items()
                      if value != defaults.get(name)}
            if differ:
                groups[group.key] = _nest(differ)
        return groups

    async def save(self) -> None:
        """Keep the settings that differ from their defaults for the bot's next start; written in a worker thread,
        as flushing to the card can take the control loop's time."""
        if self.file is None:
            raise TuningRefused("Nowhere to save: this bot was started without a tuning file", status=409)
        async with self.saving:
            groups, stamp = self.to_save(), datetime.now().isoformat(timespec="seconds")
            text = json.dumps({"saved_at": stamp, "groups": groups}, indent=2) + "\n"
            await asyncio.to_thread(write_whole, self.file, text)
            self.saved, self.saved_at, self.problems = groups, stamp, []  # The skipped settings are not in the new file
        logger.info(f"Saved tuning to {self.file}: {json.dumps(groups)}")
