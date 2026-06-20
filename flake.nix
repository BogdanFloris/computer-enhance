{
  description = "Computer Enhance - Performance Engineering";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, flake-utils }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = nixpkgs.legacyPackages.${system};
      in
      {
        devShells.default = pkgs.mkShell {
          buildInputs = with pkgs; [
            clang
            clang-tools
            cmake
            gtest
            nasm
            ninja
            lldb
          ];

          # The nixpkgs lldb has no code-signed `debugserver`, so it can't
          # launch processes on macOS (`run` fails with "executable doesn't
          # exist: '(empty)'"). Point it at the signed one shipped with the
          # Command Line Tools / Xcode.
          shellHook = ''
            for ds in \
              /Library/Developer/CommandLineTools/Library/PrivateFrameworks/LLDB.framework/Resources/debugserver \
              /Applications/Xcode.app/Contents/SharedFrameworks/LLDB.framework/Resources/debugserver; do
              if [ -x "$ds" ]; then
                export LLDB_DEBUGSERVER_PATH="$ds"
                break
              fi
            done
          '';
        };
      });
}
