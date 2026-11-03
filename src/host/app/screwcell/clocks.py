"""Read-only PL 32-bit microsecond tick mapping to bracketed host monotonic ns."""
class PLClock:
    def __init__(self):
        self.previous=None;self.extended=0;self.epoch=0
    def sample(self,tick,before_ns,after_ns):
        if type(tick) is not int or not 0<=tick<1<<32 or not 0<=before_ns<=after_ns:
            raise ValueError('invalid clock sample')
        midpoint=(before_ns+after_ns)//2;reason=None
        if self.previous is None:
            self.extended=tick;reason='INITIAL_SAMPLE'
        else:
            prior_tick,prior_ns=self.previous;elapsed_us=(midpoint-prior_ns)/1000
            delta=(tick-prior_tick)&0xffffffff
            # Bound the gap to make wrap extension unambiguous. A reset/reload
            # or unexpected rate change begins a new software mapping epoch.
            if not 0<=elapsed_us<=60_000_000 or abs(delta-elapsed_us)>max(5000,elapsed_us*.002):
                self.epoch+=1;self.extended=tick;reason='RESET_RATE_OR_SAMPLING_GAP_REMAP'
            else:self.extended+=delta
        self.previous=(tick,midpoint)
        return dict(pl_tick_us=tick,extended_tick_us=self.extended,software_mapping_epoch=self.epoch,
            host_before_ns=before_ns,host_after_ns=after_ns,
            offset_host_minus_pl_ns=midpoint-self.extended*1000,
            bracket_uncertainty_ns=(after_ns-before_ns+1)//2+1000,
            continuity_verified=reason is None,remap_reason=reason,
            scope='Bracketed software register read, no hardware PTP or exposure synchronization')
