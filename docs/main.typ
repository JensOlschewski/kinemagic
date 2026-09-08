#set document(
  title: "Kinemagic Solver Documentation",
  author: "Jens Olschewski",
)

#set page(
  paper: "a4",
  margin: (x: 25mm, y: 25mm),
  numbering: "1",
)
#set text(size: 10.5pt)
#set par(justify: true)
#set heading(numbering: "1.1")
#set math.equation(numbering: "(1)")


#align(center)[
  #v(25%)
  #text(size: 24pt, weight: "bold")[Kinemagic]
  #v(8pt)
  #text(size: 16pt)[Solver Documentation]
  #v(20pt)
  Technical and mathematical formulation
  #v(1fr)
  Jens Olschewski
]

#pagebreak()

#outline(title: [Contents], depth: 3)

#pagebreak()

= Introduction <introduction>

Kinemagic models and solves multibody mechanisms. This document complements
the Rust API documentation by describing the mathematical conventions, solver
concepts, and their relationship to the implementation.

The current solver supports ground-rooted trees of rigid bodies connected by
spherical joints. Body poses are propagated from ground along a spanning tree.
Closed-loop position analysis, additional joint types, and elastokinematics are
future extensions and will be identified as such where discussed.

#include "notation.typ"

#bibliography("references.bib", style: "ieee")
