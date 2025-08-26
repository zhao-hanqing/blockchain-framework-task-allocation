from collections.abc import Mapping, Sequence

def _safe_import_types():
    try:
        from toychain.src.Block import TaskTree, TaskNode  # adjust path if needed
        return TaskTree, TaskNode
    except Exception:
        return None, None

def _is_tasknode(obj):
    return (
        hasattr(obj, "name")
        and hasattr(obj, "reports")
        and hasattr(obj, "seps")
        and hasattr(obj, "children")
        and isinstance(getattr(obj, "children", None), dict)
    )

def _is_tasktree(obj):
    return hasattr(obj, "root") and _is_tasknode(getattr(obj, "root"))

def to_wire(obj):
    """Recursively convert objects to pickle/JSON-safe primitives with tags for custom types."""
    # Lazy imports for optional deps
    try:
        import sympy as sp
    except Exception:
        sp = None

    # Try to import concrete types
    TaskTree, TaskNode = _safe_import_types()

    # TaskNode
    if (TaskNode and isinstance(obj, TaskNode)) or _is_tasknode(obj):
        return {
            "__type__": "TaskNode",
            "name": to_wire(obj.name),
            "reports": to_wire(obj.reports),
            "seps": to_wire(obj.seps),
            "children": {str(k): to_wire(v) for k, v in getattr(obj, "children", {}).items()},
        }

    # TaskTree
    if (TaskTree and isinstance(obj, TaskTree)) or _is_tasktree(obj):
        return {
            "__type__": "TaskTree",
            "root": to_wire(obj.root),
        }

    # ---------- SymPy scalars ----------
    if sp and isinstance(obj, sp.Basic):
        try:
            return float(obj)
        except Exception:
            return str(obj)

    # ---------- SymPy matrices ----------
    if sp and hasattr(sp, "MatrixBase") and isinstance(obj, sp.MatrixBase):
        return [[float(x) for x in row] for row in obj.tolist()]

    # ---------- Mappings ----------
    if isinstance(obj, Mapping):
        return {to_wire(k): to_wire(v) for k, v in obj.items()}

    # ---------- Sequences (but not bytes/str) ----------
    if isinstance(obj, (list, tuple, set)):
        return [to_wire(x) for x in obj]

    # ---------- Primitives ----------
    if isinstance(obj, (bytes, str, int, float, bool, type(None))):
        return obj

    # ---------- Fallback via __getstate__ ----------
    if hasattr(obj, "__getstate__"):
        return to_wire(obj.__getstate__())

    # ---------- Strict fallback ----------
    raise TypeError(f"to_wire: unsupported type {type(obj).__name__} -> {repr(obj)[:200]}")


def from_wire(obj):
    if isinstance(obj, (bytes, str, int, float, bool, type(None))):
        return obj

    if isinstance(obj, list):
        return [from_wire(x) for x in obj]

    if isinstance(obj, dict):
        t = obj.get("__type__")
        if t == "TaskNode":
            # local import to avoid circulars
            from toychain.src.Block import TaskNode  # adjust path if needed
            node = TaskNode(
                name=from_wire(obj["name"]),
                reports=from_wire(obj["reports"]),
                seps=from_wire(obj["seps"]),
            )

            children_dict = obj.get("children", {})
            node.children = {k: from_wire(v) for k, v in children_dict.items()}
            return node

        if t == "TaskTree":
            try:
                from toychain.src.Block import TaskTree  # adjust path if needed
                tt = TaskTree.__new__(TaskTree)  # bypass __init__
            except Exception:
                tt = type("TaskTreeShim", (), {})()
            tt.root = from_wire(obj["root"])
            return tt

        # Plain dict (no tag)
        return {from_wire(k): from_wire(v) for k, v in obj.items()}

    # Unknown types
    return obj