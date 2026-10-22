import hashlib
import importlib.util
import json
from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parent
WORKSPACE = ROOT.parents[1]
source = ROOT.parent / 'documentation_roundup_20261020_r1/document_audit.py'
specification = importlib.util.spec_from_file_location('old_readonly_document_audit', source)
module = importlib.util.module_from_spec(specification)
specification.loader.exec_module(module)
DOCUMENTS = ('HANDOFF_CURRENT.md', 'CENTRIFUGE_FULL_CYCLE.md', 'QUICKSTART.md',
             'HANDOFF_PROMPT.md', 'NEXT_CHAT_HANDOFF.md', 'README.md')


def save(name, data):
    with (ROOT / name).open('x') as stream:
        json.dump(data, stream, indent=2, ensure_ascii=False)


def main():
    if sys.argv[1:] == ['before']:
        records = {}
        for name in (*DOCUMENTS, 'AGENTS.md'):
            data = (WORKSPACE / name).read_bytes()
            records[name] = dict(bytes=len(data), sha256=hashlib.sha256(data).hexdigest())
            if name != 'AGENTS.md':
                records[name]['text'] = data.decode()
        save('documentation_before.json', records)
        save('protected_before.json', module.protect())
        save('live_before.json', module.live())
        return
    assert sys.argv[1:] == ['after']
    before = json.loads((ROOT / 'documentation_before.json').read_text())
    authority = before.pop('AGENTS.md')
    current = (WORKSPACE / 'AGENTS.md').read_bytes()
    assert len(current) > authority['bytes']
    assert hashlib.sha256(current[:authority['bytes']]).hexdigest() == authority['sha256']
    save('live_after.json', module.live())
    records, links = {}, []
    for name, old in before.items():
        assert hashlib.sha256(old['text'].encode()).hexdigest() == old['sha256']
        data = (WORKSPACE / name).read_bytes()
        contents = data.decode()
        assert 'r352' in contents and 'r333' in contents, name
        assert len(re.findall(r'^```', contents, re.MULTILINE)) % 2 == 0
        for destination in re.findall(r'\[[^\]\n]*\]\(([^)\n]+)\)', contents):
            if '://' not in destination and not destination.startswith('#'):
                relative = destination.split('#', 1)[0]
                assert (WORKSPACE / relative).exists(), (name, relative)
                links.append([name, relative])
        records[name] = dict(bytes=len(data), sha256=hashlib.sha256(data).hexdigest())
    protected = module.protect()
    assert protected == json.loads((ROOT / 'protected_before.json').read_text())
    save('protected_after.json', protected)
    save('documentation_after.json', records)
    save('documentation_validation.json', dict(status='PASS', original_authority_prefix=authority,
        prefix_unchanged=True, checked_links=len(links), bounded_protection_only=True,
        all_runtime_protection_separately_recorded=True))


if __name__ == '__main__':
    main()
