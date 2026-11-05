import importlib.util
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("barcode_map", ROOT / "tools/barcode_map.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class MappingTests(unittest.TestCase):
    def test_every_letter_can_be_assigned_to_every_action(self):
        for i in range(26):
            values = [chr(65 + (i + j) % 26) for j in range(4)]
            mapping = dict(zip(module.ACTIONS, values))
            payload, ack_letters = module.mapping_payload(mapping)
            self.assertEqual(json.loads(payload), mapping)
            self.assertEqual(ack_letters.decode(), "".join(values))
            self.assertLessEqual(len(payload), 128)

    def test_invalid_or_ambiguous_mappings(self):
        for values in [("A", "A", "B", "C"), ("a", "B", "C", "D"),
                       ("AA", "B", "C", "D"), ("*", "B", "C", "D"),
                       (None, "B", "C", "D")]:
            with self.assertRaises(ValueError):
                module.mapping_payload(dict(zip(module.ACTIONS, values)))
        for mapping in [{}, {"forward": "A"}, dict.fromkeys((*module.ACTIONS, "extra"), "A")]:
            with self.assertRaises(ValueError):
                module.mapping_payload(mapping)


if __name__ == "__main__":
    unittest.main()
