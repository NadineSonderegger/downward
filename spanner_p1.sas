begin_version
3
end_version
begin_metric
0
end_metric
9
begin_variable
var0
-1
3
Atom at(bob, location1)
Atom at(bob, location2)
Atom at(bob, location3)
end_variable
begin_variable
var1
-1
2
Atom at(spanner3, location2)
Atom carrying(bob, spanner3)
end_variable
begin_variable
var2
-1
2
Atom at(spanner2, location2)
Atom carrying(bob, spanner2)
end_variable
begin_variable
var3
-1
2
Atom at(spanner1, location2)
Atom carrying(bob, spanner1)
end_variable
begin_variable
var4
-1
2
Atom useable(spanner1)
NegatedAtom useable(spanner1)
end_variable
begin_variable
var5
-1
2
Atom useable(spanner2)
NegatedAtom useable(spanner2)
end_variable
begin_variable
var6
-1
2
Atom useable(spanner3)
NegatedAtom useable(spanner3)
end_variable
begin_variable
var7
-1
2
Atom loose(nut1)
Atom tightened(nut1)
end_variable
begin_variable
var8
-1
2
Atom loose(nut2)
Atom tightened(nut2)
end_variable
0
begin_state
0
0
0
0
0
0
0
0
0
end_state
begin_goal
2
7 1
8 1
end_goal
11
begin_operator
pickup_spanner location2 spanner1 bob
1
0 1
1
0 3 0 1
1
end_operator
begin_operator
pickup_spanner location2 spanner2 bob
1
0 1
1
0 2 0 1
1
end_operator
begin_operator
pickup_spanner location2 spanner3 bob
1
0 1
1
0 1 0 1
1
end_operator
begin_operator
tighten_nut location3 spanner1 bob nut1
2
0 2
3 1
2
0 7 0 1
0 4 0 1
1
end_operator
begin_operator
tighten_nut location3 spanner1 bob nut2
2
0 2
3 1
2
0 8 0 1
0 4 0 1
1
end_operator
begin_operator
tighten_nut location3 spanner2 bob nut1
2
0 2
2 1
2
0 7 0 1
0 5 0 1
1
end_operator
begin_operator
tighten_nut location3 spanner2 bob nut2
2
0 2
2 1
2
0 8 0 1
0 5 0 1
1
end_operator
begin_operator
tighten_nut location3 spanner3 bob nut1
2
0 2
1 1
2
0 7 0 1
0 6 0 1
1
end_operator
begin_operator
tighten_nut location3 spanner3 bob nut2
2
0 2
1 1
2
0 8 0 1
0 6 0 1
1
end_operator
begin_operator
walk location1 location2 bob
0
1
0 0 0 1
1
end_operator
begin_operator
walk location2 location3 bob
0
1
0 0 1 2
1
end_operator
0
