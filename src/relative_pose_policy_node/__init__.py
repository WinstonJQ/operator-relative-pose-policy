import sys
from pathlib import Path


def _vendored_forge_tool_src() -> str | None:
    """Locate the repo-checkout ``src`` dir containing the vendored forge_tool."""
    for parent in Path(__file__).resolve().parents:
        candidate = parent / "src" / "forge_tool" / "__init__.py"
        if candidate.is_file():
            return str(candidate.parent.parent)
    return None


# In a repo checkout the forge_tool protocol is vendored at src/forge_tool; in a
# frozen executable it is already bundled and importable.
_VENDORED_SRC = _vendored_forge_tool_src()
if _VENDORED_SRC is not None and _VENDORED_SRC not in sys.path:
    sys.path.insert(0, _VENDORED_SRC)


from .config import (
    MultiGroupRelativePosePolicyConfig,
    RelativePosePolicyConfig,
    config_from_mapping,
    load_config,
)
from .query import (
    RELATIVE_DESCRIPTOR,
    RELATIVE_ENDPOINT_ID,
    RELATIVE_OPERATION,
    RelativePoseQuery,
    RelativePoseQueryEndpoint,
)
from .resolver import (
    MultiGroupRelativePoseResolver,
    RelativePoseCommand,
    RelativePoseResolution,
    RelativePoseResolutionError,
    RelativePoseResolver,
)
from .runner import RelativePosePolicyRunner, run_relative_policy

__all__ = [
    "RELATIVE_DESCRIPTOR",
    "RELATIVE_ENDPOINT_ID",
    "RELATIVE_OPERATION",
    "MultiGroupRelativePosePolicyConfig",
    "MultiGroupRelativePoseResolver",
    "RelativePosePolicyConfig",
    "RelativePosePolicyRunner",
    "RelativePoseCommand",
    "RelativePoseQuery",
    "RelativePoseQueryEndpoint",
    "RelativePoseResolution",
    "RelativePoseResolutionError",
    "RelativePoseResolver",
    "config_from_mapping",
    "load_config",
    "run_relative_policy",
]
