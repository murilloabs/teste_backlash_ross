"""Backlash com a montagem linear equivalente a ``lambda K0: K0`` do ROSS."""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path


def _load_base_backlash():
    source = Path(__file__).resolve().parents[1] / "mestrado" / "backlash.py"
    if not source.exists():
        raise FileNotFoundError(f"Implementação-base não encontrada: {source}")
    spec = spec_from_file_location("dissertation_backlash_base", source)
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_base = _load_base_backlash()
_DissertationBacklash = _base.Backlash
compute_dfft = _base.compute_dfft


def _keep_rotor_stiffness_only(K0):
    """Retorna K0 sem adicionar a matriz linear de acoplamento da malha."""
    return K0


class Backlash(_DissertationBacklash):
    """Backlash da dissertação usando a rigidez global dos rotores separados."""

    def __init__(self, *args, **kwargs):
        use_coupling = kwargs.get("use_multirotor_coupling_stiffness", False)
        if len(args) >= 9:
            use_coupling = args[8]

        super().__init__(*args, **kwargs)
        self.multirotor.orientation_angle = self.multirotor.mesh.orientation_angle

        if use_coupling:
            self.multirotor.add_coupling_stiffness = self.multirotor.K_mesh
        else:
            self.multirotor.add_coupling_stiffness = _keep_rotor_stiffness_only
            self.multirotor.update_mesh_stiffness = False


__all__ = ["Backlash", "compute_dfft"]
