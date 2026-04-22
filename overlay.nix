final: prev:
{
  ik-solver = final.callPackage ././ik_solver/package.nix {};
  stewart-helpers = final.callPackage ././stewart_helpers/package.nix {};
  stewart-interfaces = final.callPackage ././stewart_interfaces/package.nix {};
  vision-system = final.callPackage ././vision_system/package.nix {};
}
