"""Activate the historical trace view only for an explicitly configured child."""

import os
import sys

_registry = os.environ.get('H1_TRACE_ARCHIVE_REGISTRY')
if _registry:
    try:
        from compat import install
        install(_registry)
    except BaseException as error:
        # Python otherwise reports a sitecustomize error and continues unpatched.
        sys.stderr.write(f'TRACE_ARCHIVE_BOOTSTRAP_FAILED: {error}\n')
        sys.stderr.flush()
        os._exit(78)
