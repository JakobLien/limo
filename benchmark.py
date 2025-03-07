import datetime


class Benchmark():
    def __init__(self):
        self.times = {}
        self.current = None
        self.currentStart = None
        self.ordering = []

    def stop(self):
        if not self.current:
            return
        timeDelta = datetime.datetime.now() - self.currentStart
        self.times[self.current] = timeDelta.total_seconds()
        self.current = None
        self.currentStart = None

    def start(self, name):
        if self.current:
            self.stop()
        if name not in self.ordering:
            self.ordering.append(name)
        self.current = name
        self.currentStart = datetime.datetime.now()

    def __str__(self):
        total = sum(list(self.times.values()))
        strRep = f'Total time: {round(total*1000)} ms'
        for name, time in self.times.items():
            strRep += f'\n{name}: {round(time/total * 100, 1)}%'
        strRep += '\n'
        return strRep