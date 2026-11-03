import unittest
from workcell.order_language import correct_sizes, normalize, fast_parse, correct_destinations, trays_named, split_skip_cooling, understand, friendly_error
from workcell.tasks import result


def steps(task):
    return [(s['part_class'], s['quantity'], s['destination']) for s in task['steps']]


class Language(unittest.TestCase):
    def test_long_geometry_traceback_shows_actionable_cause(self):
        error = '/some/long/report/path/' * 30 + '\\nValueError: Unlocalised nut_m6 (geometry rejected) 136 px from a target/slot: move or remove it'
        message = friendly_error(error)
        self.assertIn('小螺母', message)
        self.assertIn('136', message)
        self.assertIn('重复识别', message)
        self.assertNotIn('/some/', message)

    def test_tray_words(self):
        self.assertEqual(normalize('放到左盘'), '放到B盒');self.assertEqual(normalize('从右边的托盘拿'), '从A盒拿')
        self.assertEqual(trays_named('把螺丝放到左边托盘'), ['B'])

    def test_simple_order_is_parsed_by_rule(self):
        self.assertEqual(steps(fast_parse('把两颗螺丝放到B盒')), [('screw', 2, 'B')])
        self.assertEqual(steps(fast_parse('把三个螺丝放到左盘')), [('screw', 3, 'B')])

    def test_two_step_order(self):
        task = fast_parse('先把一颗螺母放到A盒，然后把两颗螺丝放到B盒')
        self.assertEqual(steps(task), [('nut', 1, 'A'), ('screw', 2, 'B')])

    def test_kit_order(self):
        self.assertEqual(steps(fast_parse('配一套：两颗螺丝，一颗螺母，放左盘')), [('screw', 2, 'B'), ('nut', 1, 'B')])

    def test_two_parts_in_one_clause(self):
        self.assertEqual(steps(fast_parse('帮我配一套：两颗螺丝一颗螺母，放左盘')), [('screw', 2, 'B'), ('nut', 1, 'B')])

    def test_a_part_word_without_quantity_is_left_to_the_model(self):
        self.assertIsNone(fast_parse('把两颗螺丝和螺母放到B盒'))

    def test_skip_cooling_phrases(self):
        for phrase in ('不用等降温，把两颗螺丝放到B盒', '把两颗螺丝放到B盒，不用等待降温结束', '直接开始，把两颗螺丝放到左盘', '别等降温了，把两颗螺丝放到B盒', '跳过降温把两颗螺丝放到B盒'):
            text, skip = split_skip_cooling(phrase)
            self.assertTrue(skip, phrase);self.assertEqual(steps(fast_parse(text)), [('screw', 2, 'B')], (phrase, text))

    def test_no_skip_phrase_no_flag(self):
        self.assertFalse(split_skip_cooling('把两颗螺丝放到B盒')[1])

    def test_understand_carries_the_flag(self):
        r = understand('不用等降温，把两颗螺丝放到左盘')
        self.assertTrue(r['skip_cooling']);self.assertEqual(steps(r['task']), [('screw', 2, 'B')])

    def test_definite_tray_reference_means_all_of_that_tray(self):
        task = fast_parse('把左边盘子的螺母放回右边，同时把右边盘子的螺丝放过去')
        self.assertEqual(steps(task), [('nut', 'all', 'A'), ('screw', 'all', 'B')])
        self.assertEqual([s['region'] for s in task['steps']], ['left', 'right'])

    def test_all(self):
        self.assertEqual(steps(fast_parse('把全部螺丝放到B盒')), [('screw', 'all', 'B')])

    def test_exclusion_is_not_a_step(self):
        task = fast_parse('把两颗螺丝放到B盒，螺母不要动')
        self.assertEqual(steps(task), [('screw', 2, 'B')]);self.assertEqual(task['exclude'], ['nut'])

    def test_what_rules_cannot_judge_goes_to_the_model(self):
        for text in ('把长的螺丝放到B盒', '把左边的螺丝放到B盒', '动作慢一点把两颗螺丝放到B盒', '把螺丝放到B盒'):
            self.assertIsNone(fast_parse(text), text)

    def test_model_answer_for_the_wrong_tray_is_overridden(self):
        wrong = result('plan', [dict(part_class='screw', quantity=2, region='left', destination='A')])
        fixed, changed = correct_destinations(wrong, '帮我配两颗螺丝放左盘')
        self.assertTrue(changed);self.assertEqual(fixed['steps'][0]['destination'], 'B')

    def test_two_named_trays_are_left_to_the_model(self):
        task = result('plan', [dict(part_class='nut', quantity=1, region='all', destination='A'), dict(part_class='screw', quantity=2, region='all', destination='B')])
        self.assertEqual(correct_destinations(task, '螺母放A盒，螺丝放B盒'), (task, False))



def parsed(text):
    task = fast_parse(text)
    return None if task is None else steps(task)


class FourClasses(unittest.TestCase):
    """M12 and M6 parts (2026-10-31): size words, M6/M12 names, 'all parts', and the sentence correcting the model's size."""
    def test_size_words(self):
        self.assertEqual(parsed('把所有零件放到左盘'), [('screw', 'all', 'B'), ('nut', 'all', 'B'), ('screw_m6', 'all', 'B'), ('nut_m6', 'all', 'B')])
        self.assertEqual(parsed('两颗小螺丝放到B盒'), [('screw_m6', 2, 'B')])
        self.assertEqual(parsed('一个大螺母和一个小螺母放到左盘'), [('nut', 1, 'B'), ('nut_m6', 1, 'B')])
        self.assertEqual(parsed('一颗M12螺丝、一个M12螺母、一颗M6螺丝、一个M6螺母放到左盘'),
                         [('screw', 1, 'B'), ('nut', 1, 'B'), ('screw_m6', 1, 'B'), ('nut_m6', 1, 'B')])
        self.assertEqual(parsed('两颗螺丝放到B盒'), [('screw', 2, 'B')])           # a bare part word is still the M12 part

    def test_m6_is_not_six(self):
        self.assertEqual(parsed('两颗M6螺丝放到B盒'), [('screw_m6', 2, 'B')])
        self.assertIsNone(parsed('把M6螺丝放到B盒'))                                # no quantity: never "6 screws"

    def test_exclusion_of_one_size(self):
        task = fast_parse('所有零件放到B盒，小螺母不要动')
        self.assertEqual([s['part_class'] for s in task['steps']], ['screw', 'nut', 'screw_m6'])
        self.assertEqual(task['exclude'], ['nut_m6'])

    def test_other_attributes_go_to_the_model(self):
        self.assertIsNone(parsed('两颗大的螺丝放B盒'))
        self.assertIsNone(parsed('两颗长螺丝放到B盒'))

    def test_model_size_corrected_from_the_sentence(self):
        model = result('plan', [dict(part_class='screw', quantity=2, region='all', destination='B')])
        task, changed = correct_sizes(model, '两颗小螺丝放到B盒')
        self.assertTrue(changed);self.assertEqual(task['steps'][0]['part_class'], 'screw_m6')
        with self.assertRaises(ValueError):
            correct_sizes(model, '两颗小螺母放到B盒')                               # the kind differs: ask, do not guess


if __name__ == '__main__':unittest.main()
