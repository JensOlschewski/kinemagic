use super::spherical::SphericalCoordinates;

pub enum JointCoordinates {
    Spherical(SphericalCoordinates),
    // Later:
    // Revolute(RevoluteCoordinates),
    // Prismatic(PrismaticCoordinates),
    // Planar(PlanarCoordinates),
    // Inplane(InplaneCoordinates),
    // Inline(InlineCoordinates),
    // Universal(UniversalCoordinates),
}
