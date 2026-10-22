import json
from pathlib import Path
import sys

from release_descent import plan_descent


def main():
    request = json.loads(Path(sys.argv[1]).read_text())
    result = plan_descent(request)
    with Path(sys.argv[2]).open('x') as stream:
        json.dump(result, stream, indent=2)
    assert result['status'] == 'PASS', result
    print('RELEASE_DESCENT_PASS', result['descent_m'], result['checked_samples'], flush=True)


if __name__ == '__main__':
    main()
