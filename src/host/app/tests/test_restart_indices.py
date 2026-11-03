"""A repeated pause must retain checkpoint indices across numeric worker IPC."""
import pickle
import unittest
from screwcell.motion_paths import restart_entries,validate_restart_indices

class RestartIndicesTests(unittest.TestCase):
    def test_second_pause_uses_explicit_indices_after_independent_ipc(self):
        bank=[dict(words=[i]) for i in range(8)]
        checkpoint=dict(resume=[dict(words=[99])],suffix=[dict(words=[98])]+bank[4:],suffix_original_indices=[None,4,5,6,7])
        # ProcessPool results and the main bank are independent object graphs.
        checkpoint=pickle.loads(pickle.dumps(checkpoint));bank=pickle.loads(pickle.dumps(bank))
        entries=restart_entries(checkpoint)
        self.assertIsNot(entries[2][0],bank[4]);self.assertEqual([i for _,i in entries],[None,None,4,5,6,7])
        available={5:'next_brake',7:'final_rest'}
        self.assertEqual(next(i for _,i in entries if i in available),5)
        validate_restart_indices(dict(records=bank),checkpoint)
    def test_rest_restart_keeps_identical_hold_records_at_their_own_indices(self):
        a=dict(words=[0]);checkpoint=dict(resume=[],suffix=[a,a,a],suffix_original_indices=[4,5,6])
        self.assertEqual([i for _,i in restart_entries(pickle.loads(pickle.dumps(checkpoint)))],[4,5,6])
        checkpoint['suffix_original_indices']=[6,5,4]
        with self.assertRaisesRegex(ValueError,'increase'):restart_entries(checkpoint)
    def test_increasing_but_wrong_original_index_is_rejected_disabled(self):
        bank=dict(records=[dict(words=[0]),dict(words=[1])])
        checkpoint=dict(resume=[],suffix=[dict(words=[0])],suffix_original_indices=[1])
        with self.assertRaisesRegex(ValueError,'does not match'):validate_restart_indices(bank,checkpoint)
