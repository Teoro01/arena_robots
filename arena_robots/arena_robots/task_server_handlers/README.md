# Task kinds

A **task kind** is a public action endpoint the `task_server` advertises on
behalf of a robot. Each `TaskKind` has one action type (defined in
`arena_robots_msgs`) and one public suffix (e.g. `goto_pose`); the
`task_server` mounts it under the robot's namespace as
`<namespace>/<suffix>`. Which task kinds a running `task_server` actually
advertises depends on the selected `bringup` and the handlers registered for
that `(TaskKind, bringup_kind)` pair.

## Sources of truth

| File | Role |
|---|---|
| [`task_kinds.py`](../task_kinds.py) | `TaskKind` enum, `PUBLIC_SUFFIX`, `action_type()`, `endpoint()`, the only place these are defined |
| `task_server_handlers/__init__.py` | `TaskHandler` ABC + `_executor_sleep` helper. No registry: handler ownership lives on the Bringup subclass. |
| `task_server_handlers/<kind>/__init__.py` | exports the per-`TaskKind` handler type alias (e.g. `GotoPoseHandler`) for handler implementations |
| `task_server_handlers/<kind>/<bringup>.py` | the `TaskHandler` implementation for that `(kind, bringup)` pair |
| `bringup/<cap>/<kind>.py` | declares `task_handlers: ClassVar[dict[TaskKind, Callable[[], type]]]` mapping each supported `TaskKind` to a zero-arg loader for the handler class (lazy msgs imports) |
| `arena_robots_msgs/action/<Kind>.action` | the IDL the action type comes from |
| `clients/<kind>.py` | optional Python client wrapper used by `task_generator` and standalone drivers |

## Adding a new task kind

### 1. Define the action IDL

Add `arena_robots_msgs/action/<Kind>.action`. Keep the goal/feedback/result
fields minimal and framework-neutral; every bringup has to implement it, so
nothing bringup-specific belongs here.

### 2. Register the enum and suffix

In [`task_kinds.py`](../task_kinds.py):

```python
class TaskKind(enum.Enum):
    GOTO_POSE = "goto_pose"
    FOLLOW_PATH = "follow_path"  # new


PUBLIC_SUFFIX: dict[TaskKind, str] = {
    TaskKind.GOTO_POSE: "goto_pose",
    TaskKind.FOLLOW_PATH: "follow_path",  # new
}


def action_type(tk: TaskKind) -> type:
    if tk is TaskKind.GOTO_POSE:
        from arena_robots_msgs.action import GotoPose

        return GotoPose
    if tk is TaskKind.FOLLOW_PATH:  # new
        from arena_robots_msgs.action import FollowPath

        return FollowPath
    raise KeyError(tk)
```

The import stays inside the branch so `arena_robots_msgs` is only resolved
when the kind is actually in use.

### 3. Create the handler package

```
task_server_handlers/
└── follow_path/
    ├── __init__.py         # exports FollowPathHandler type alias
    ├── nav2.py             # FollowPathHandlerNav2(TaskHandler)
    └── _passthrough.py     # optional: shared implementations for none/external
```

`task_server_handlers/follow_path/__init__.py`:

```python
from arena_robots_msgs.action import FollowPath

from arena_robots.task_server_handlers import TaskHandler

FollowPathHandler = TaskHandler[FollowPath.Goal, FollowPath.Feedback, FollowPath.Result]
```

A `TaskHandler` is an `ABC` (see the base registry module): subclass it,
accept `(bringup, *, tf_buffer, node)` in `__init__`, and implement
`async def execute(goal_handle) -> Result`. Override `on_cancel` if you need
custom cancel handling; the base returns `CancelResponse.ACCEPT`.

### 4. (Optional) Ship a Python client

Mirror `clients/goto_pose.py` as `clients/follow_path.py`. Clients are not
required (any consumer can talk to the raw action endpoint), but `task_generator`
and [DRIVING.md](../../../DRIVING.md) examples use them.

### 5. Wire into a `Bringup`

Each `Bringup` subclass declares a `task_handlers` ClassVar mapping `TaskKind`
to a zero-arg loader for the handler class. Adding support for a new task kind
in an existing bringup is one entry in that dict:

```python
def _load_follow_path_nav2() -> type:
    from arena_robots.task_server_handlers.follow_path.nav2 import FollowPathHandlerNav2

    return FollowPathHandlerNav2


@BringupMeta.attach(requires={"mobile"}, cap="mobile")
class Nav2Bringup(Bringup):
    kind = "nav2"
    task_handlers: ClassVar[dict] = {
        TaskKind.GOTO_POSE: _load_goto_pose_nav2,
        TaskKind.FOLLOW_PATH: _load_follow_path_nav2,
    }
```

The `task_server` iterates `bringup.accepts_task_kinds` (derived from
`task_handlers.keys()`) at startup and invokes the loader to construct the
handler. A new task kind is visible as soon as at least one bringup adds an
entry.

## Design invariants

- **`TaskKind` is closed.** The enum is the sole allowlist; `task_generator`,
  the `task_server`, and clients all key off it. Don't parametrise the set by
  config.
- **Handler ownership lives on the `Bringup`.** The bringup is the single
  source of truth for `kind`, launch actions, and supported `TaskKind`s.
  Adding a new bringup without a matching handler is a missing-key bug, not
  a silent hang.
- **Loaders stay zero-arg and lazy.** Putting the `nav2_msgs` import (or any
  other non-core msgs import) at module top level will break bringups that
  don't need it.
- **No fallback handlers.** A bringup either declares a `TaskKind` in
  `task_handlers` or it doesn't; `task_server` skips advertising that
  endpoint when missing instead of silently degrading.
