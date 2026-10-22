def restore_loaded_dispatch_source(content):
    text = content.decode()
    changes = (
        ('    def maximum_loaded_rate(self, phase):\n        return .25\n\n    def move(self, samples, phase, rate):',
         '    def move(self, samples, phase, rate):'),
        ('if self.holders.rows and rate > self.maximum_loaded_rate(phase):',
         'if self.holders.rows and rate > .25:'),
        ("raise ValueError('Paired loaded rate cannot exceed its declared phase cap')",
         "raise ValueError('Paired loaded rate cannot exceed candidate quarter rad/s')"))
    for new, old in changes:
        if text.count(new) != 1:
            raise ValueError('Unexpected additional loaded-dispatch edit')
        text = text.replace(new, old, 1)
    return text.encode()
