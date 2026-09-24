# Comparação das matrizes de rigidez da malha

Use o kernel `Python (ross3)`. Para uma comparação direta, comece pelo notebook unificado:

0. `00_comparacao_unificada_3_metodologias.ipynb`

Os notebooks individuais podem ser usados para inspecionar cada metodologia separadamente:

1. `01_matriz_backlash_py.ipynb`
2. `02_matriz_backlash_ross.ipynb`
3. `03_matriz_backlash_nativo_ross.ipynb`

Cada notebook constrói o mesmo multirrotor e extrai os DOFs globais das duas engrenagens. A comparação principal é:

```text
Kgear - K0gear
```

onde `K0gear` é formado somente pelas matrizes dos rotores. O notebook também calcula a contribuição esperada:

```text
K_coupling * mesh.stiffness
```

Interpretação esperada:

- `backlash.py`: `Kgear - K0gear` deve coincidir com `K_coupling * mesh.stiffness`;
- `backlash_ross.py`: `Kgear - K0gear` deve ser zero;
- backlash nativo do ROSS: `Kgear - K0gear` deve ser zero.

Os notebooks não executam a integração dinâmica; eles ativam/construem cada modalidade e inspecionam a matriz linear global.
