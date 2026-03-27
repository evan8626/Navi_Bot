#!/usr/bin/env python3
"""
Real-Time Perforamnce Profiler

Monitor execution time, deadline misses, and resource usage.
"""

import time
from collections import defaultdict, deque
import numpy as np

class PerforamnceProfiler:
    """
    Tracks execution time and deadline compliance for real-time tasks
    """
    
    def __init__(self, history_size=100):
        self.history_size = history_size

        # Timing data: {task_name: deque of execution times}
        self.execution_times = defaultdict(lambda: deque(maxlen=history_size))

        # Deadline tracking: {task_name: (deadline, miss_count)}
        self.deadlines = {}

        # Active timers: {task_name: start_time}
        self.active_timers = {}

    def start_timing(self, task_name):
        """Start timing a task"""
        self.active_timers[task_name] = time.perf_counter()
    
    def stop_timing(self, task_name, deadline = None):
        """
        Stop timing a task and record execution time.

        Args:
            task_name: Name of the task
            deadline: Optional deadline in seconds
        
        Returns:
            Execution time in seconds
        """
        if task_name not in self.active_timers:
            return None
        
        elapsed = time.perf_counter() - self.active_timers[task_name]
        del self.active_timers[task_name]

        # Record execution time
        self.execution_times[task_name].append(elapsed)

        # Check deadline
        if deadline is not None:
            if task_name not in self.deadlines:
                self.deadlines[task_name] = [deadline, 0]
            
            if elapsed > deadline:
                self.deadlines[task_name][1] += 1
        
        return elapsed

    def get_statistics(self, task_name):
        """
        Get performance statistics for a task.

        Returns:
            Dictionary with mean, max, min, std, deadline misses
        """
        if task_name not in self.execution_times:
            return None
        
        times = List(self.execution_times[task_name])

        stats = {
            'mean': np.mean(times),
            'max': np.max(times),
            'min': np.min(times),
            'std': np.std(times),
            'count': len(times)
        }

        if task_name in self.deadlines:
            deadline, misses = self.deadlines[task_name]
            stats['deadline'] = deadline
            stats['deadline_misses'] = misses
            stats['deadline_miss_rate'] = misses / len(times)

        return stats
    
    def print_report(self):
        """Print performance report for all tasks."""
        print("\n" + "="*60)
        print("PERFORMANCE REPORT")
        print("="*60)

        for task_name in self.execution_times.keys():
            stats = self.get_statistics(task_name)
            print(f"\nTask: {task_name}")
            print(f"  Mean:    {stats['mean']*1000:.2f} ms")
            print(f"  Max:     {stats['max']*1000:.2f} ms")
            print(f"  Min:     {stats['min']*1000:.2f} ms")
            print(f"  Std:     {stats['std']*1000:.2f} ms")
            print(f"  Samples: {stats['count']}")

            if "deadline" in stats:
                print(f"  Deadline:  {stats['deadline']*1000:.2f} ms")
                print(f"  Misses:    {stats['deadline_misses']}")
                print(f"  Miss Rate: {stats['deadline_miss_rate']*100}%")
    
    def reset(self, task_name=None):
        """Reset statistics for a task or all tasks."""
        if task_name is None:
            self.execution_times.clear()
            self.deadlines.clear()
        else:
            if task_name in self.execution_times:
                del self.execution_times[task_name]
            if task_name in self.deadlines:
                del self.deadlines[task_name]

class Timer:
    """Context manager for easy timing"""

    def __init__(self, profiler, task_name, deadline=None):
        self.profiler = profiler
        self.task_name = task_name
        self.deadline = deadline
    
    def __enter__(self):
        self.profiler.start_timing(self.task_name)
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.profiler.stop_timing(self.task_name, self.deadline)

# Global profiler instance
global_profiler = PerforamnceProfiler()

def profile(task_name, deadline = None):
    """ 
    Decorator for profiling function execution.

    Usage:
        @profile('control_loop', deadline=0.02)
        def control_loop():
            # ...
    """
    def decorator(func):
        def wrapper(*args, **kwargs):
            global_profiler.start_timing(task_name)
            result = func(*args, **kwargs)
            global_profiler.stop_timing(task_name, deadline)
            return result
        return wrapper
    return decorator