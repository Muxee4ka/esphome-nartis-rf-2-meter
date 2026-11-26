"""Корпус считывателя Нартис: ESP32 DevKit V1 (30 pin) + CC1101 с SMA.

Раскладка (вид сверху, X вдоль корпуса):

    [USB]  ESP32 пинами вверх  | провода |  CC1101 пинами вверх  [SMA -> антенна]

Обе платы лежат компонентами вниз, штыри вверх, на них сверху надеты
Dupont-мамы; над ними запас на изгиб проводов. Крышка надевается сверху
(юбка внутрь, посадка с зазором). Печать без поддержек: коробка дном вниз,
крышка верхней гранью вниз.

Запуск: pip install build123d && python nartis_case.py  (пересоберёт STL/STEP рядом)
"""

from pathlib import Path

from build123d import (
    Align,
    Axis,
    Box,
    BuildPart,
    BuildSketch,
    Cone,
    Cylinder,
    Locations,
    Mode,
    Plane,
    Polygon,
    Pos,
    RectangleRounded,
    SlotOverall,
    chamfer,
    export_step,
    export_stl,
    extrude,
    fillet,
)

OUT = Path(__file__).parent

# ---------------- платы (мерить штангенциркулем!) ----------------
ESP_L, ESP_W = 51.5, 28.5       # ESP32 DevKit V1 30 pin, USB-C, CH340
PCB_T = 1.6
UNDER_H = 6.0                   # под платой: модуль ESP-WROOM 3.1 + хвосты пинов + USB
HEADER_H = 2.5                  # пластик штыревой линейки
DUPONT_H = 14.5                 # корпус Dupont-мамы
WIRE_BEND = 10.0                # запас над разъёмами на изгиб проводов

CC_L, CC_W = 28.0, 15.0         # CC1101 (E07-M1101D / «CC1101 V2.0»), чертёж A x B
CC_HOLE_X = 19.4                # центр монтажных отверстий от края с пинами
CC_HOLE_Y = 2.7                 # центр отверстий от длинного края (оба края)
CC_SMA_Y = 6.6                  # ось SMA от длинного края (8.4 от другого, чертёж I)
SMA_BODY = 1.5                  # от края платы CC1101 до начала резьбы SMA
SMA_HOLE_D = 7.2                # резьба SMA 1/4"-36 (6.35) + зазор

USB_W, USB_H = 13.0, 12.0       # окно под штекер USB-C (высокое: подходит при любой стороне разъёма)

# ---------------- корпус ----------------
WALL = 2.0
FLOOR = 2.0
FIT = 0.5                       # зазор вокруг плат
GAP_BOARDS = 12.0               # между ESP32 и CC1101 (там идут провода)
FILLET_R = 2.5
FOOT_CHAMFER = 0.4

LID_T = 2.0
LIP_DEPTH = 6.0                 # v2: было 4.0
LIP_T = 1.2
LID_CLEAR = 0.1                 # v2: было 0.2 — крышка болталась

# v2: удержание крышки без переделки коробки
RIB_R = 0.6                     # вертикальные рёбра натяга на длинных сторонах юбки
RIB_INTERF = 0.2                # насколько гребень ребра заходит за внутреннюю стенку (натяг на сторону)
RIB_XS = (-28.0, -10.0, 10.0, 28.0)
SNAP = True                     # защёлки в крайние вентщели коробки (со стороны USB)
FINGER_W, FINGER_T, FINGER_SLIT = 3.6, 1.2, 1.0
FINGER_GAP = 0.15               # зазор пальца до стенки
BUMP_W, BUMP_H = 1.6, 1.0       # зуб: ширина вдоль щели (щель 2.0), вылет
SNAP_CLEAR = 0.2                # зуб ниже верхнего края щели

PAD = 4.0                       # угловые опоры ESP32, мм
GUIDE_T, GUIDE_H = 1.2, 2.0     # уголки-фиксаторы ESP32 выше платы
CC_POST_D = 6.0
SELF_TAP_D = 2.7                # M3 саморез (необязательно)

TAB = True                      # ушки для крепления на стену
TAB_L, TAB_W, TAB_T, TAB_HOLE = 12.0, 9.0, 3.0, 4.5

VENT_W, VENT_L, VENT_PITCH = 2.0, 14.0, 4.0

# ---------------- производные ----------------
BOARD_Z = FLOOR + UNDER_H                       # низ обеих плат
INNER_L = FIT + ESP_L + GAP_BOARDS + CC_L + SMA_BODY
INNER_W = ESP_W + 2 * FIT
INNER_H = UNDER_H + PCB_T + HEADER_H + DUPONT_H + WIRE_BEND
BOX_L, BOX_W = INNER_L + 2 * WALL, INNER_W + 2 * WALL
BOX_H = FLOOR + INNER_H

X0 = -INNER_L / 2                               # внутренний торец со стороны USB
ESP_X0 = X0 + FIT
ESP_X1 = ESP_X0 + ESP_L
ESP_Y0 = -ESP_W / 2
CC_X0 = ESP_X1 + GAP_BOARDS                     # край CC1101 со стороны пинов
CC_X1 = CC_X0 + CC_L
CC_Y0 = -CC_W / 2
SMA_Y = CC_Y0 + CC_SMA_Y
SMA_Z = BOARD_Z + PCB_T / 2
VENT_Z = BOARD_Z + PCB_T + HEADER_H + 3 + VENT_L / 2          # центр вентщелей коробки
VENT_XS = [ESP_X0 + 3 + VENT_PITCH / 2 + i * VENT_PITCH for i in range(int((ESP_L - 6) // VENT_PITCH))]
SNAP_X = VENT_XS[0]                                           # крайняя щель у USB — над ней нет проводов


def build_box():
    with BuildPart() as box:
        Box(BOX_L, BOX_W, BOX_H, align=(Align.CENTER, Align.CENTER, Align.MIN))
        fillet(box.edges().filter_by(Axis.Z), FILLET_R)
        # полость
        with Locations(Pos(0, 0, FLOOR)):
            Box(INNER_L, INNER_W, INNER_H + 1, align=(Align.CENTER, Align.CENTER, Align.MIN),
                mode=Mode.SUBTRACT)
        chamfer(box.edges().group_by(Axis.Z)[0], FOOT_CHAMFER)

        # --- ESP32: угловые опоры; по Y и со стороны USB плату держат стенки (зазор FIT),
        #     у дальнего края — два упора, чтобы не уезжала к CC1101 ---
        for sx in (0, 1):
            for sy in (0, 1):
                px = ESP_X0 + (ESP_L - PAD if sx else 0) + PAD / 2
                py = ESP_Y0 + (ESP_W - PAD if sy else 0) + PAD / 2
                with Locations(Pos(px, py, FLOOR - 0.01)):
                    Box(PAD, PAD, UNDER_H + 0.01, align=(Align.CENTER, Align.CENTER, Align.MIN))
        for sy in (0, 1):
            py = ESP_Y0 + (ESP_W - PAD if sy else 0) + PAD / 2
            with Locations(Pos(ESP_X1 + FIT + GUIDE_T / 2, py, FLOOR - 0.01)):
                Box(GUIDE_T, PAD + FIT, UNDER_H + PCB_T + GUIDE_H + 0.01,
                    align=(Align.CENTER, Align.CENTER, Align.MIN))

        # --- CC1101: две стойки под отверстиями + опорное ребро под краем с пинами ---
        for hy in (CC_Y0 + CC_HOLE_Y, CC_Y0 + CC_W - CC_HOLE_Y):
            with Locations(Pos(CC_X0 + CC_HOLE_X, hy, FLOOR - 0.01)):
                Cylinder(CC_POST_D / 2, UNDER_H + 0.01, align=(Align.CENTER, Align.CENTER, Align.MIN))
        with Locations(Pos(CC_X0 + 0.5, 0, FLOOR - 0.01)):
            Box(1.0, CC_W, UNDER_H + 0.01, align=(Align.CENTER, Align.CENTER, Align.MIN))
        # упор перед ребром, чтобы плата не уезжала к ESP
        with Locations(Pos(CC_X0 - FIT - 0.6, 0, FLOOR - 0.01)):
            Box(1.2, CC_W, UNDER_H + PCB_T + 1.5, align=(Align.CENTER, Align.CENTER, Align.MIN))

        # отверстия под саморезы в стойках CC1101
        for hy in (CC_Y0 + CC_HOLE_Y, CC_Y0 + CC_W - CC_HOLE_Y):
            with Locations(Pos(CC_X0 + CC_HOLE_X, hy, BOARD_Z + 0.01)):
                Cylinder(SELF_TAP_D / 2, UNDER_H - 1.0, align=(Align.CENTER, Align.CENTER, Align.MAX),
                         mode=Mode.SUBTRACT)

        # --- окно USB ---
        with Locations(Pos(-BOX_L / 2, 0, SMA_Z)):
            Box(2 * WALL + 1, USB_W, USB_H, mode=Mode.SUBTRACT)
        # --- отверстие SMA ---
        with Locations(Pos(BOX_L / 2, SMA_Y, SMA_Z)):
            Cylinder(SMA_HOLE_D / 2, 2 * WALL + 1, rotation=(0, 90, 0), mode=Mode.SUBTRACT)

        # --- вентиляция: вертикальные щели в длинных стенках, выше плат ---
        for side in (-1, 1):
            for x in VENT_XS:
                with Locations(Pos(x, side * BOX_W / 2, VENT_Z)):
                    Box(VENT_W, 2 * WALL + 1, VENT_L, mode=Mode.SUBTRACT)
    part = box.part

    if TAB:
        with BuildPart() as tabs:
            for sx in (-1, 1):
                with Locations(Pos(sx * (BOX_L / 2 + TAB_L / 2 - 1), 0, 0)):
                    Box(TAB_L + 2, TAB_W, TAB_T, align=(Align.CENTER, Align.CENTER, Align.MIN))
            fillet(tabs.edges().filter_by(Axis.Z).group_by(Axis.X)[0] + tabs.edges().filter_by(Axis.Z).group_by(Axis.X)[-1], 2.0)
            for sx in (-1, 1):
                with Locations(Pos(sx * (BOX_L / 2 + TAB_L / 2), 0, TAB_T / 2)):
                    Cylinder(TAB_HOLE / 2, TAB_T + 1, mode=Mode.SUBTRACT)
        part = part + tabs.part
    return part


def build_lid():
    """Крышка v2. Печатается как моделируется: пластина на столе, юбка вверх.
    z=LID_T в координатах крышки = верхний торец коробки; глубже в коробку = больше z."""
    in_y = INNER_W / 2
    lo_l, lo_w = INNER_L - 2 * LID_CLEAR, INNER_W - 2 * LID_CLEAR
    with BuildPart() as lid:
        Box(BOX_L, BOX_W, LID_T, align=(Align.CENTER, Align.CENTER, Align.MIN))
        fillet(lid.edges().filter_by(Axis.Z), FILLET_R)
        chamfer(lid.edges().group_by(Axis.Z)[0], FOOT_CHAMFER)
        # юбка внутрь корпуса
        with BuildSketch(Plane.XY.offset(LID_T)):
            RectangleRounded(lo_l, lo_w, 0.8)
            RectangleRounded(lo_l - 2 * LIP_T, lo_w - 2 * LIP_T, 0.4, mode=Mode.SUBTRACT)
        extrude(amount=LIP_DEPTH)

        # рёбра натяга на длинных сторонах (там юбка гибкая), с заходной фаской на конце
        rib_y = in_y + RIB_INTERF - RIB_R
        lead = 1.2
        for side in (-1, 1):
            for x in RIB_XS:
                with Locations(Pos(x, side * rib_y, LID_T - 0.01)):
                    Cylinder(RIB_R, LIP_DEPTH - lead + 0.01, align=(Align.CENTER, Align.CENTER, Align.MIN))
                with Locations(Pos(x, side * rib_y, LID_T + LIP_DEPTH - lead)):
                    Cone(RIB_R, RIB_R - RIB_INTERF - LID_CLEAR, lead,
                         align=(Align.CENTER, Align.CENTER, Align.MIN))

        if SNAP:
            fx1 = SNAP_X + BUMP_W / 2 + 0.6          # палец сдвинут к торцу USB, подальше от штырей
            fx0 = fx1 - FINGER_W
            face = in_y - FINGER_GAP                 # наружная грань пальца
            # прорезь в юбке вокруг пальца, чтобы он пружинил
            for side in (-1, 1):
                with Locations(Pos((fx0 + fx1) / 2, side * (in_y - 1.0), LID_T)):
                    Box(FINGER_W + 2 * FINGER_SLIT, 2.4, LIP_DEPTH + 1,
                        align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)
            # зуб: 45° снизу (печать без поддержек и снимается усилием), заход 45° на конце
            slot_top = LID_T + (BOX_H - (VENT_Z + VENT_L / 2))   # верх щели в координатах крышки
            zc = slot_top + SNAP_CLEAR + (BUMP_H - FINGER_GAP)   # всё, что за стенкой, — ниже края щели
            z_end = zc + 0.3 + BUMP_H
            for side in (-1, 1):
                with Locations(Pos((fx0 + fx1) / 2, side * (face - FINGER_T / 2), LID_T - 0.01)):
                    Box(FINGER_W, FINGER_T, z_end - LID_T + 0.01,
                        align=(Align.CENTER, Align.CENTER, Align.MIN))
                pts = [(face - 0.01, zc - BUMP_H), (face + BUMP_H, zc), (face + BUMP_H, zc + 0.3),
                       (face - 0.01, z_end)]
                pts = [(side * y, z) for y, z in pts]
                with BuildSketch(Plane.YZ.offset(SNAP_X - BUMP_W / 2)):
                    Polygon(*pts, align=None)
                extrude(amount=BUMP_W)

        # прорези над ESP32
        with BuildSketch(Plane.XY.offset(-0.01)):
            with Locations(*[(x, 0) for x in
                             [ESP_X0 + 6 + i * VENT_PITCH for i in range(int((ESP_L - 10) // VENT_PITCH) + 1)]]):
                SlotOverall(INNER_W - 10, VENT_W, rotation=90)
        extrude(amount=LID_T + 0.02, mode=Mode.SUBTRACT)
    return lid.part


if __name__ == "__main__":
    box = build_box()
    lid = build_lid()
    for name, p in (("nartis_box", box), ("nartis_lid", lid)):
        export_stl(p, str(OUT / f"{name}.stl"))
        export_step(p, str(OUT / f"{name}.step"))
        bb = p.bounding_box()
        print(f"{name}: {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm, V={p.volume:.0f} mm3")
    print(f"inner {INNER_L:.1f} x {INNER_W:.1f} x {INNER_H:.1f}; board bottom z={BOARD_Z}, SMA axis z={SMA_Z:.1f} y={SMA_Y:.1f}")
