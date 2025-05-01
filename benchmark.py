import time

class Benchmark():
    def __init__(self):
        self.times = {}
        self.current = None
        self.currentStart = None

    def stop(self):
        if not self.current:
            return
        self.times[self.current] = time.perf_counter() - self.currentStart
        self.current = None
        self.currentStart = None

    def start(self, name: str):
        if self.current:
            self.stop()
        self.current = name
        self.currentStart = time.perf_counter()

    def __str__(self):
        self.stop()
        total = sum(list(self.times.values()))
        strRep = f'Total time: {round(total*1000)} ms'
        for name, time in self.times.items():
            strRep += f'\n{name}: {round(time/total * 100, 1)}%'
        strRep += '\n'
        return strRep