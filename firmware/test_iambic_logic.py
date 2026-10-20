#!/usr/bin/env python3
"""Host-side simulation of EtherKeyer Mini Curtis A / Accu-Keyer B memory logic."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List


class Mode(Enum):
    A = auto()
    B = auto()


class Element(Enum):
    IDLE = auto()
    DIT = auto()
    DAH = auto()


@dataclass
class Keyer:
    mode: Mode
    dit_memory: bool = False
    dah_memory: bool = False
    dit_pad: bool = False
    dah_pad: bool = False
    dit_rose: bool = False
    dah_rose: bool = False
    last: Element = Element.IDLE
    output: List[str] = field(default_factory=list)

    def arm_during(self, current: Element) -> None:
        """Arm opposite memory during element or its following gap (same rules)."""
        if current == Element.DIT:
            if self.mode == Mode.A:
                if self.dah_rose:
                    self.dah_memory = True
            elif self.dah_pad:
                self.dah_memory = True
        elif current == Element.DAH:
            if self.mode == Mode.A:
                if self.dit_rose:
                    self.dit_memory = True
            elif self.dit_pad:
                self.dit_memory = True
        # Mode A drops latched memory when both paddles are up (squeeze release).
        if self.mode == Mode.A and not self.dit_pad and not self.dah_pad:
            self.dit_memory = False
            self.dah_memory = False
        self.dit_rose = False
        self.dah_rose = False

    def decide(self) -> Element:
        next_el = Element.IDLE
        if self.last == Element.DIT:
            if self.dah_memory or self.dah_pad:
                next_el = Element.DAH
            elif self.dit_pad:
                next_el = Element.DIT
        elif self.last == Element.DAH:
            if self.dit_memory or self.dit_pad:
                next_el = Element.DIT
            elif self.dah_pad:
                next_el = Element.DAH
        return next_el

    def send(self, el: Element) -> None:
        if el == Element.DIT:
            self.dit_memory = False
            self.output.append(".")
        else:
            self.dah_memory = False
            self.output.append("-")
        self.last = el

    def run_element(self, el: Element, *, dit_pad: bool, dah_pad: bool, dit_rose: bool = False, dah_rose: bool = False) -> None:
        self.dit_pad = dit_pad
        self.dah_pad = dah_pad
        self.dit_rose = dit_rose
        self.dah_rose = dah_rose
        self.send(el)
        self.arm_during(el)
        # Inter-element gap: Mode B may still level-arm; Mode A only on a new edge
        self.dit_rose = False
        self.dah_rose = False
        self.arm_during(el)  # gap uses same opposite-of-last rules via last==el after send

    def finish_after_gap(self) -> None:
        nxt = self.decide()
        while nxt != Element.IDLE:
            # Trailing elements with paddles released (Mode B bonus, or memory inserts)
            self.run_element(nxt, dit_pad=self.dit_pad, dah_pad=self.dah_pad)
            if self.dit_pad or self.dah_pad:
                # Continue while held; for release tests pads are cleared by caller before finish
                nxt = self.decide()
            else:
                nxt = self.decide()
                if nxt != Element.IDLE and not (self.dit_memory or self.dah_memory):
                    # Only memory can queue with pads up
                    break


def letter_c_mode_a() -> str:
    """Squeeze dah-first (dit edge during first dah); release both during final dit."""
    k = Keyer(Mode.A)
    k.run_element(Element.DAH, dit_pad=True, dah_pad=True, dit_rose=True)
    k.run_element(Element.DIT, dit_pad=True, dah_pad=True)
    k.run_element(Element.DAH, dit_pad=True, dah_pad=True)
    # Final dit; release both during this dit
    k.run_element(Element.DIT, dit_pad=False, dah_pad=False)
    k.dit_pad = False
    k.dah_pad = False
    assert k.decide() == Element.IDLE
    return "".join(k.output)


def letter_c_mode_b() -> str:
    """Squeeze dah-first; release both during second dah → bonus dit completes C."""
    k = Keyer(Mode.B)
    k.run_element(Element.DAH, dit_pad=True, dah_pad=True)
    k.run_element(Element.DIT, dit_pad=True, dah_pad=True)
    # Second dah: level-arms dit_memory; release both during/after this dah
    k.run_element(Element.DAH, dit_pad=True, dah_pad=True)
    k.dit_pad = False
    k.dah_pad = False
    # Gap after second dah still Mode-B level-armed dit_memory from element
    assert k.dit_memory is True
    nxt = k.decide()
    assert nxt == Element.DIT
    k.run_element(Element.DIT, dit_pad=False, dah_pad=False)
    k.dit_pad = False
    k.dah_pad = False
    assert k.decide() == Element.IDLE
    return "".join(k.output)


def mode_a_unsqueeze_keep_dit() -> str:
    """After alternating, release dah but keep dit → continue dits, no trailing dah."""
    k = Keyer(Mode.A)
    k.run_element(Element.DIT, dit_pad=True, dah_pad=True)
    # Unsqueeze dah (no Mode A edge), keep dit
    k.dit_pad = True
    k.dah_pad = False
    k.dah_memory = False
    nxt = k.decide()
    assert nxt == Element.DIT
    k.run_element(Element.DIT, dit_pad=True, dah_pad=False)
    return "".join(k.output)


def mode_a_gap_release_no_extra() -> str:
    """Mode A: continuous squeeze then release during gap → no bonus element."""
    k = Keyer(Mode.A)
    k.run_element(Element.DAH, dit_pad=True, dah_pad=True)
    # Release during gap after dah; no edge, no level latch in A
    k.dit_pad = False
    k.dah_pad = False
    k.arm_during(Element.DAH)  # gap
    assert k.dit_memory is False
    assert k.decide() == Element.IDLE
    return "".join(k.output)


def mode_b_gap_release_bonus_dit() -> str:
    """Mode B: squeeze through dah, release in gap → bonus dit."""
    k = Keyer(Mode.B)
    k.run_element(Element.DAH, dit_pad=True, dah_pad=True)
    k.dit_pad = False
    k.dah_pad = False
    # dit_memory already armed by level during dah
    assert k.dit_memory is True
    assert k.decide() == Element.DIT
    k.run_element(Element.DIT, dit_pad=False, dah_pad=False)
    assert k.decide() == Element.IDLE
    return "".join(k.output)


def mode_a_short_squeeze_no_trailer() -> str:
    """Mode A: dit closes during dah, then both released — no trailing dit."""
    k = Keyer(Mode.A)
    k.run_element(Element.DAH, dit_pad=True, dah_pad=True, dit_rose=True)
    k.dit_pad = False
    k.dah_pad = False
    k.arm_during(Element.DAH)
    assert k.dit_memory is False
    assert k.decide() == Element.IDLE
    return "".join(k.output)


def mode_b_short_squeeze_trailer() -> str:
    """Mode B: same short squeeze releases during the dah and still sends the dit."""
    k = Keyer(Mode.B)
    k.run_element(Element.DAH, dit_pad=True, dah_pad=True, dit_rose=True)
    k.dit_pad = False
    k.dah_pad = False
    k.arm_during(Element.DAH)
    assert k.dit_memory is True
    assert k.decide() == Element.DIT
    k.run_element(Element.DIT, dit_pad=False, dah_pad=False)
    assert k.decide() == Element.IDLE
    return "".join(k.output)


def mode_a_mid_element_tap_inserts_dit() -> str:
    """Mode A: during dah, rising edge on dit inserts dit even if tip released before gap."""
    k = Keyer(Mode.A)
    k.run_element(Element.DAH, dit_pad=False, dah_pad=True, dit_rose=True, dah_rose=False)
    assert k.dit_memory is True
    k.dit_pad = False
    k.dah_pad = True  # still holding dahs
    assert k.decide() == Element.DIT
    k.run_element(Element.DIT, dit_pad=False, dah_pad=True)
    return "".join(k.output)


def main() -> None:
    tests = [
        ("Mode A letter C", letter_c_mode_a, "-.-."),
        ("Mode B letter C", letter_c_mode_b, "-.-."),
        ("Mode A unsqueeze keep dit", mode_a_unsqueeze_keep_dit, ".."),
        ("Mode A gap release", mode_a_gap_release_no_extra, "-"),
        ("Mode B gap bonus dit", mode_b_gap_release_bonus_dit, "-."),
        ("Mode A edge insert dit", mode_a_mid_element_tap_inserts_dit, "-."),
        ("Mode A short squeeze", mode_a_short_squeeze_no_trailer, "-"),
        ("Mode B short squeeze", mode_b_short_squeeze_trailer, "-."),
    ]
    failed = 0
    for name, fn, expected in tests:
        got = fn()
        ok = got == expected
        print(f"{'PASS' if ok else 'FAIL'}: {name}: got {got!r} expected {expected!r}")
        if not ok:
            failed += 1
    if failed:
        raise SystemExit(failed)
    print(f"All {len(tests)} iambic logic tests passed.")


if __name__ == "__main__":
    main()
