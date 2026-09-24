"""Backlash da dissertação com a montagem de rigidez usada pelo ROSS atual.

Este módulo preserva a formulação e o integrador definidos em ``backlash.py``.
A única mudança estrutural é a remoção da matriz linear de acoplamento da malha
quando ``use_multirotor_coupling_stiffness=False``. Nesse caso, ``MultiRotor.K``
retorna apenas a matriz global montada a partir das matrizes dos dois rotores.
"""

try:
    from .backlash import Backlash as _DissertationBacklash
    from .backlash import compute_dfft
except ImportError:
    from backlash import Backlash as _DissertationBacklash
    from backlash import compute_dfft


__all__ = ["Backlash", "compute_dfft"]


def _keep_rotor_stiffness_only(K0):
    """Retorna a matriz dos rotores sem adicionar a matriz de acoplamento."""
    return K0


class Backlash(_DissertationBacklash):
    """Backlash com remoção efetiva da rigidez linear da malha.

    Os parâmetros são os mesmos de :class:`backlash.Backlash`. Quando
    ``use_multirotor_coupling_stiffness`` é falso, a função usada por
    ``MultiRotor.K`` é substituída por uma identidade, reproduzindo a abordagem
    ``lambda K0: K0`` do backlash nativo do ROSS. A rigidez armazenada em
    ``multirotor.mesh.stiffness`` permanece disponível para a força não linear.
    """

    def __init__(
        self,
        multirotor,
        speed_driving_gear,
        b0=0.0,
        error_amp=0.0,
        gear_mesh_stiffness=None,
        num_points_cicle=1000,
        n_cicles=2,
        cut_cicles=1,
        use_multirotor_coupling_stiffness=False,
        compute_contact_ratio=True,
        mesh_damping_ratio=0.07,
    ):
        super().__init__(
            multirotor=multirotor,
            speed_driving_gear=speed_driving_gear,
            b0=b0,
            error_amp=error_amp,
            gear_mesh_stiffness=gear_mesh_stiffness,
            num_points_cicle=num_points_cicle,
            n_cicles=n_cicles,
            cut_cicles=cut_cicles,
            use_multirotor_coupling_stiffness=use_multirotor_coupling_stiffness,
            compute_contact_ratio=compute_contact_ratio,
            mesh_damping_ratio=mesh_damping_ratio,
        )

        # Compatibilidade com chamadas da implementação da dissertação. No ROSS
        # atual, o ângulo de orientação pertence ao objeto Mesh.
        self.multirotor.orientation_angle = self.multirotor.mesh.orientation_angle

        if use_multirotor_coupling_stiffness:
            self.multirotor.add_coupling_stiffness = self.multirotor.K_mesh
        else:
            # MultiRotor.K monta K0 com as matrizes dos rotores nas posições
            # globais corretas e passa o resultado por esta função identidade.
            self.multirotor.add_coupling_stiffness = _keep_rotor_stiffness_only
            self.multirotor.update_mesh_stiffness = False

