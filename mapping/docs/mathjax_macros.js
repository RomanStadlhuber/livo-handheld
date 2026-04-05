// MathJax 3 macro definitions for LIVO Handheld Mapping documentation.
// Loaded via MATHJAX_CODEFILE in Doxyfile.in before MathJax renders.
MathJax = {
  tex: {
    macros: {

      // --- Generic bold shorthands ---
      // Note: \vec is reserved by MathJax for the arrow-hat accent, so use \bvec.
      mtx:  ['{\\boldsymbol{#1}}', 1],   // bold matrix,  e.g. \mtx{A}
      bvec: ['{\\boldsymbol{#1}}', 1],   // bold vector,  e.g. \bvec{v}

      // --- World ↔ IMU ---
      wTi: '{{}^{w}\\boldsymbol{T}_{I}}',
      wRi: '{{}^{w}\\boldsymbol{R}_{I}}',
      wpi: '{{}^{w}\\boldsymbol{p}_{I}}',

      // --- World ↔ LiDAR ---
      wTl: '{{}^{w}\\boldsymbol{T}_{L}}',
      wRl: '{{}^{w}\\boldsymbol{R}_{L}}',
      wpl: '{{}^{w}\\boldsymbol{p}_{L}}',

      // --- IMU ↔ LiDAR (extrinsic calibration) ---
      iTl: '{{}^{I}\\boldsymbol{T}_{L}}',
      iRl: '{{}^{I}\\boldsymbol{R}_{L}}',
      ipl: '{{}^{I}\\boldsymbol{p}_{L}}',
      lTi: '{{}^{L}\\boldsymbol{T}_{I}}',
      lRi: '{{}^{L}\\boldsymbol{R}_{I}}',
      lpi: '{{}^{L}\\boldsymbol{p}_{I}}',

      // --- Skew-symmetric / Lie algebra ---
      // SO(3) skew-symmetric matrix:  \skewOf{\omg}  →  [ω]×
      skewOf:   ['{\\bigl[{#1}\\bigr]_{\\times}}', 1],
      // SE(3) twist matrix (hat map): \twistHat{\tau} →  [τ]
      twistHat: ['{\\bigl[{#1}\\bigr]}', 1],

      // --- Kinematic quantities ---
      omg:    '{\\boldsymbol{\\omega}}',   // angular velocity  ω
      linVel: '{\\boldsymbol{\\rho}}',     // linear velocity   ρ
      Ohm:    '{\\boldsymbol{\\Omega}}',   // uppercase Omega   Ω

      // --- State transition matrix ---
      // \PhiOf{x}  →  Φ(x)
      PhiOf: ['{\\boldsymbol{\\Phi}(#1)}', 1],

      // --- Delta pose ---
      dT: '{\\Delta\\boldsymbol{T}}',
    }
  }
};
