import time
from navi_bot.utils.profiler import PerformanceProfiler, profile, global_profiler, Timer
p=PerformanceProfiler(history_size=5)
for _ in range(8):
    p.start_timing('t'); p.stop_timing('t', deadline=0.0)
print("miss rate after 8 misses, history 5:", p.get_statistics('t')['deadline_miss_rate'])
p=PerformanceProfiler()
p.start_timing('t'); p.stop_timing('t', deadline=0.0); p.start_timing('t'); p.stop_timing('t', deadline=10.0)
print("reported deadline after 0.0 then 10.0:", p.get_statistics('t')['deadline'])
p=PerformanceProfiler(); p.start_timing('t'); p.stop_timing('t', deadline=1.0); p.start_timing('t'); p.stop_timing('t')
print("one call with deadline + one without -> miss_rate denominator count:", p.get_statistics('t')['count'])
@profile('boom')
def boom(): raise ValueError('x')
try: boom()
except ValueError: pass
print("after exception: samples", len(global_profiler.execution_times.get('boom', [])), "leaked active timer:", 'boom' in global_profiler.active_timers)
@profile('named')
def my_func():
    """doc"""
print("decorated __name__:", my_func.__name__, "__doc__:", my_func.__doc__)
pp=PerformanceProfiler()
with __import__('contextlib').suppress(ValueError):
    with Timer(pp,'ctx'):
        raise ValueError
print("Timer records sample on exception:", len(pp.execution_times['ctx']))
# zero-elapsed probability
zeros=0
for _ in range(200000):
    q=PerformanceProfiler(); q.start_timing('a'); e=q.stop_timing('a', deadline=0.0)
    if e<=0: zeros+=1
print("elapsed<=0 occurrences in 200k start/stop:", zeros)
# perf_counter resolution
import time; print("perf_counter info:", time.get_clock_info('perf_counter'))
e=PerformanceProfiler(); _=e.execution_times['ghost']
try: e.print_report()
except Exception as ex: print("print_report after defaultdict read ->", type(ex).__name__, ex)
