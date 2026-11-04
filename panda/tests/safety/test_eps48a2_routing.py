"""Compile the production safety-mode selector with mocked board I/O; no hardware."""
import ctypes
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]


class TestEps48A2Routing(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    main = (ROOT / "board/main.c").read_text()
    selector = main[main.index("void set_safety_mode("):main.index("\nbool is_car_safety_mode(")]
    definitions = []
    for path, prefix in ((ROOT.parent / "opendbc_repo/opendbc/safety/declarations.h", "#define SAFETY_"),
                         (ROOT / "board/boards/board_declarations.h", "#define CAN_MODE_")):
      definitions.extend(line for line in path.read_text().splitlines() if line.startswith(prefix))
    harness = r"""
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
static bool honda_eps48a2_enabled, heartbeat_lost, can_silent, relay;
static bool reject_mode;
static uint32_t safety_tx_blocked, safety_rx_invalid, heartbeat_counter;
static int last_can_mode, cleared_bus;
static void board_mode(uint8_t mode) { last_can_mode = mode; }
static struct { void (*set_can_mode)(uint8_t); } board = {board_mode}, *current_board = &board;
static int set_safety_hooks(uint16_t mode, uint16_t param) {
  (void)param;
  return (reject_mode && (mode != SAFETY_SILENT)) ? -1 : 0;
}
static void set_intercept_relay(bool enabled, bool ignition) { relay = enabled; (void)ignition; }
static void can_clear_send(int iface, int bus) { (void)iface; cleared_bus = bus; }
static void can_init_all(void) { }
static void print(const char *message) { (void)message; }
#define CANIF_FROM_CAN_NUM(n) (n)
#define assert_fatal(condition, message) assert(condition)
"""
    wrapper = r"""
int apply_mode(uint16_t mode, uint16_t param, bool diagnostic_enabled, bool reject) {
  honda_eps48a2_enabled = diagnostic_enabled;
  reject_mode = reject;
  last_can_mode = -1;
  cleared_bus = -1;
  set_safety_mode(mode, param);
  return last_can_mode;
}
int get_cleared_bus(void) { return cleared_bus; }
bool get_relay(void) { return relay; }
"""
    cls.directory = tempfile.TemporaryDirectory(prefix="eps48a2-routing-")
    cls.addClassCleanup(cls.directory.cleanup)
    source = Path(cls.directory.name) / "selector.c"
    library = source.with_suffix(".so")
    source.write_text("\n".join(definitions) + harness + selector + wrapper)
    subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-shared", "-fPIC", "-fsanitize=undefined",
                    "-fno-sanitize-recover=all", str(source), "-o", str(library)], check=True)
    cls.lib = ctypes.CDLL(str(library))
    cls.lib.apply_mode.argtypes = [ctypes.c_uint16, ctypes.c_uint16, ctypes.c_bool, ctypes.c_bool]
    cls.lib.apply_mode.restype = ctypes.c_int
    cls.lib.get_cleared_bus.restype = ctypes.c_int
    cls.lib.get_relay.restype = ctypes.c_bool

  def test_obd_route_is_limited_to_elm_or_enabled_honda(self):
    # Diagnostic_enabled represents the Honda hook's initialized state; the
    # complete safety suite separately checks which configurations enable it.
    for mode in range(40):
      for param in (0, 1, 8, 40):
        for enabled in (False, True):
          expected = int((mode == 3 and param == 0) or (mode == 20 and enabled))
          self.assertEqual(self.lib.apply_mode(mode, param, enabled, False), expected, (mode, param, enabled))
          self.assertEqual(self.lib.get_cleared_bus(), 1 if mode == 3 or (mode == 20 and enabled) else -1)

  def test_honda_keeps_relay_and_restores_normal_route_on_exit(self):
    self.assertEqual(self.lib.apply_mode(20, 40, True, False), 1)
    self.assertTrue(self.lib.get_relay())
    self.assertEqual(self.lib.apply_mode(19, 0, False, False), 0)
    self.assertFalse(self.lib.get_relay())
    self.assertEqual(self.lib.apply_mode(20, 8, False, False), 0)
    self.assertTrue(self.lib.get_relay())

  def test_rejected_safety_configuration_falls_back_to_normal_silent(self):
    self.assertEqual(self.lib.apply_mode(20, 40, True, True), 0)
    self.assertFalse(self.lib.get_relay())


if __name__ == "__main__":
  unittest.main()
