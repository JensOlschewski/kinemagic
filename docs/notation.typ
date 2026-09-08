// Math helper functions

// Matrix quantity
#let m(x) = math.bold(x)

// Vector with arrow
#let v(x) = math.accent(x, math.arrow)

= Notation and conventions <notation>

The notation of the kinemagic solver is strongly related to the notation of use
by Woernle and Nikravesh @nikravesh_overview_2005
@woernle_mehrkorpersysteme_2022. The following sections summarize the most
important


== Body Pose
All position vectors are column vectors in three-dimensional Euclidean space.
Unless stated otherwise, vector components are expressed in the ground frame
$cal(K)_0$. Rotation matrices map coordinates from a local frame into its parent
frame.

== Indices

#table(
  columns: (auto, 1fr),
  align: (center, left),
  inset: 7pt,
  stroke: 0.5pt,
  table.header([*Index*], [*Meaning*]),
  [$0$], [Ground body or ground frame],
  [$i, j$], [Rigid bodies],
  [$a, b$], [Markers attached to bodies],
  [$k$], [Iteration index],
)

== Frames and rigid bodies

#table(
  columns: (auto, 1fr),
  align: (center, left),
  inset: 7pt,
  stroke: 0.5pt,
  table.header([*Symbol*], [*Meaning*]),
  [$cal(K)_0$], [Ground-fixed reference frame],
  [$cal(K)_i$], [Body-fixed frame of body $i$],
  [$bold(r)_i$], [Position of the origin of $cal(K)_i$ in $cal(K)_0$],
  [$bold(R)_i$], [Rotation from $cal(K)_i$ into $cal(K)_0$],
  [$bold(q)_i$],
  [Unit quaternion representing $bold(R)_i$ in the implementation],
)

The pose of body $i$ is the pair

$
  bold(x)_i = (bold(r)_i, bold(R)_i).
$ <body-pose>

Ground has the identity pose:

$
  bold(r)_0 = bold(0), quad bold(R)_0 = bold(I)_3.
$ <ground-pose>

== Markers and joints

A marker is a frame rigidly attached to a body. For marker $a$ on body $i$:

#table(
  columns: (auto, 1fr),
  align: (center, left),
  inset: 7pt,
  stroke: 0.5pt,
  table.header([*Symbol*], [*Meaning*]),
  [$bold(s)_(i,a)$], [Marker position expressed in $cal(K)_i$],
  [$bold(A)_(i,a)$], [Marker rotation from its marker frame into $cal(K)_i$],
  [$bold(p)_(i,a)$], [Marker position expressed in $cal(K)_0$],
  [$bold(C)_(i,a)$], [Marker rotation into $cal(K)_0$],
  [$bold(Q)_(i a, j b)$],
  [Relative rotation from marker $(j,b)$ to marker $(i,a)$],
)

World-space marker pose follows from its body pose:

$
  bold(p)_(i,a) = bold(r)_i + bold(R)_i bold(s)_(i,a),
  quad
  bold(C)_(i,a) = bold(R)_i bold(A)_(i,a).
$ <marker-pose>

== Constraint and solver notation

#table(
  columns: (auto, 1fr),
  align: (center, left),
  inset: 7pt,
  stroke: 0.5pt,
  table.header([*Symbol*], [*Meaning*]),
  [$bold(q)$], [Vector of generalized coordinates],
  [$bold(Phi)(bold(q))$], [Vector of position-level constraint residuals],
  [$bold(J)(bold(q))$],
  [Constraint Jacobian $partial bold(Phi) / partial bold(q)$],

  [$Delta bold(q)$], [Coordinate correction in one nonlinear solver iteration],
  [$epsilon$], [Convergence tolerance],
)

These symbols establish documentation-wide conventions. Individual chapters
define joint-specific coordinates, residuals, and Jacobian blocks where they are
used.
