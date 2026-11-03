import unittest
import struct
from types import SimpleNamespace
from unittest.mock import patch,Mock
from continuous_transfer import initial_jaw_upper,initial_jaw_lower,TransferDevice,contract,path_digest
from direct_departure import direct_departure


class ClosedStartTests(unittest.TestCase):
    def test_fully_open_measured_start_still_requires_monotone_normalization(self):
        from test_alignment_hover import HoverContractTests
        j,p=HoverContractTests().fixture()
        j['initial_normalize_from_observed_wide']=True
        j['start_rad']=[0.]*6+[.001];j['start_motor_rad']=j['start_rad'][:]
        row=j['tasks'][0]['segments'][0]['coefficients_rad'][6]
        row[:]=[.001,1.119,0.,0.,0.,0.,0.,0.]
        j['geometry_audit']['trajectory_sha256']=path_digest(j);contract(j,p)
        self.assertAlmostEqual(initial_jaw_lower(j),-.001)
        row[1]=-1.119;j['geometry_audit']['trajectory_sha256']=path_digest(j)
        with self.assertRaises(ValueError):contract(j,p)
        d=TransferDevice(Mock());d.initial_jaw_hold=.001
        packet=lambda value:struct.pack('<HHf',0x7016,0,value)
        d.enqueue(0x1200fd07,packet(.001))
        with self.assertRaises(ValueError):d.enqueue(0x1200fd07,packet(.05))
        d.initial_jaw_hold=None
        with self.assertRaises(ValueError):d.enqueue(0x1200fd07,packet(.001))

    def test_wide_contract_normalization_and_wrong_direction(self):
        from test_alignment_hover import HoverContractTests
        j,p=HoverContractTests().fixture()
        j['initial_normalize_from_observed_wide']=True
        j['start_rad']=[0.]*6+[.91];j['start_motor_rad']=j['start_rad'][:]
        row=j['tasks'][0]['segments'][0]['coefficients_rad'][6]
        row[:]=[.91,.21,0.,0.,0.,0.,0.,0.]
        j['geometry_audit']['trajectory_sha256']=path_digest(j)
        contract(j,p)
        row[1]=-.21
        j['geometry_audit']['trajectory_sha256']=path_digest(j)
        with self.assertRaises(ValueError):contract(j,p)

    def test_wide_start_exact_hold_and_revocation(self):
        d=TransferDevice(Mock());packet=lambda value:struct.pack('<HHf',0x7016,0,value)
        with self.assertRaises(ValueError):d.enqueue(0x1200fd07,packet(.91))
        d.initial_jaw_hold=.91;d.enqueue(0x1200fd07,packet(.91))
        with self.assertRaises(ValueError):d.enqueue(0x1200fd07,packet(.95))
        d.initial_jaw_hold=None
        with self.assertRaises(ValueError):d.enqueue(0x1200fd07,packet(.91))

    def test_wide_start_must_be_measured_and_normalized(self):
        j=dict(initial_normalize_from_observed_wide=True,initial_open_during_departure=True,
               start_rad=[0.]*6+[.91],start_motor_rad=[0.]*6+[.91])
        self.assertAlmostEqual(initial_jaw_lower(j),.908)
        for change in [dict(initial_open_during_departure=False),dict(initial_open_from_observed_closed=True),
                       dict(start_rad=[0.]*6+[.95]),dict(start_motor_rad=[0.]*6+[.85])]:
            with self.assertRaises(ValueError):initial_jaw_lower(dict(j,**change))

    def test_wide_departure_only_toward_working_opening(self):
        p=SimpleNamespace(positions=lambda value:list(value));start=[0.]*6+[.91]
        with patch('direct_departure.movej_c3',return_value=[dict(duration_s=1.)]),patch('direct_departure.continuity_metrics',return_value={}):
            direct_departure(p,start,[0.]*6+[1.12],True,observed_wide_start=.91)
            for target,opening,observed in [(1.12,True,None),(.90,True,.91),(1.6,True,.91),(1.12,False,.91),(1.12,True,.95)]:
                with self.assertRaises(ValueError):direct_departure(p,start,[0.]*6+[target],opening,observed_wide_start=observed)

    def test_packet_gate_accepts_exact_start_hold_only_and_can_be_revoked(self):
        d=TransferDevice(Mock());packet=lambda value:struct.pack('<HHf',0x7016,0,value)
        with self.assertRaises(ValueError):d.enqueue(0x1200fd07,packet(1.696))
        d.initial_jaw_hold=1.696;d.enqueue(0x1200fd07,packet(1.696))
        with self.assertRaises(ValueError):d.enqueue(0x1200fd07,packet(1.698))
        d.initial_jaw_hold=None
        with self.assertRaises(ValueError):d.enqueue(0x1200fd07,packet(1.696))
    def job(self):
        return dict(initial_open_from_observed_closed=True,initial_open_during_departure=True,
                    start_rad=[0.]*6+[1.696],start_motor_rad=[0.]*6+[1.696])

    def test_exact_observed_start_allows_only_small_guard_margin(self):
        self.assertAlmostEqual(initial_jaw_upper(self.job()),1.698)
        self.assertEqual(initial_jaw_upper({}),1.62)

    def test_unobserved_mismatched_or_excess_start_rejected(self):
        for change in ['mismatch','large','no_open','nan']:
            j=self.job()
            if change=='mismatch':j['start_rad'][6]=1.65
            elif change=='large':j['start_motor_rad'][6]=j['start_rad'][6]=1.71
            elif change=='no_open':j['initial_open_during_departure']=False
            else:j['start_rad'][6]=float('nan')
            with self.assertRaises(ValueError):initial_jaw_upper(j)

    def test_departure_only_accepts_measured_monotone_opening(self):
        p=SimpleNamespace(positions=lambda value:list(value));start=[0.]*6+[1.696]
        with patch('direct_departure.movej_c3',return_value=[dict(duration_s=1.)]),patch('direct_departure.continuity_metrics',return_value={}):
            direct_departure(p,start,[0.]*6+[1.12],True,1.696)
            for target,opening,observed in [(1.12,True,None),(1.699,True,1.696),(1.696,False,1.696),(1.12,True,1.66)]:
                with self.assertRaises(ValueError):direct_departure(p,start,[0.]*6+[target],opening,observed)


if __name__=='__main__':unittest.main()
