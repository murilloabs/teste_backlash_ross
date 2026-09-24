# Análise das divergências entre o backlash da dissertação e o ROSS atual

## 1. Objetivo

Este relatório compara, por análise estática e reconstrução matemática, a implementação de engrenamento com folga usada nos códigos da dissertação com a implementação atual do ROSS. O foco é identificar diferenças capazes de explicar:

- o segundo pico observado no diagrama de bifurcação;
- o comportamento irregular próximo de 4500 rpm;
- divergências entre a resposta dos códigos da dissertação e a resposta do ROSS atual.

Não foram executados modelos, integrações, varreduras de velocidade ou scripts de validação. As conclusões são classificadas como **fato de código**, **consequência matemática** ou **hipótese causal**.

As referências de linha usam os arquivos existentes no momento da análise:

- `mestrado/backlash.py` — implementação da dissertação;
- `mestrado/bifurcation_validation.py` — varredura de bifurcação;
- `mestrado/validation_paper_01.py` — validação pontual e análise próxima de 4500 rpm;
- `ross/multi_rotor/mesh.py` — modelo atual de malha e backlash do ROSS;
- `ross/multi_rotor/multi_rotor.py` — montagem do sistema multirrotor;
- `ross/multi_rotor/utils.py` — interpolação e fórmulas auxiliares;
- `ross/rotor_assembly.py` e `ross/utils.py` — integração temporal atual.

Os três primeiros estão sob `C:/Users/M/Desktop/doutorado/teste_backlash_ross`; os arquivos do ROSS estão sob `C:/Users/M/Documents/ROSS_DEV/ross`.

## 2. Resultado principal

O mecanismo de remoção da rigidez linear da malha **não é equivalente** entre o código da dissertação e o ROSS atual.

No código da dissertação, quando `use_coupling=False`, são atribuídos:

```python
self.multirotor.gear_mesh_stiffness = 0
self.multirotor.update_mesh_stiffness = False
```

em `mestrado/backlash.py:665-667`. Entretanto, no ROSS atual a matriz de acoplamento usa `self.mesh.stiffness`, e não `self.gear_mesh_stiffness`, em `ross/multi_rotor/multi_rotor.py:763-782`. Além disso, a função que acrescenta o acoplamento já foi selecionada na construção de `MultiRotor`: quando não existe backlash nativo, `self.add_coupling_stiffness = self.K_mesh` em `ross/multi_rotor/multi_rotor.py:253-256`.

Portanto, com o código atual:

\[
\mathbf K_{\mathrm{dissertação}}
=
\mathbf K_{0}
+
\mathbf P^T\left(k_{m}\mathbf K_c\right)\mathbf P,
\]

e a força não linear de backlash é adicionada separadamente durante a integração. Isso mantém **duas vias de interação entre as engrenagens**: uma linear bilateral e outra não linear unilateral.

No ROSS nativo, quando `mesh.backlash` existe, a seleção é:

```python
self.add_coupling_stiffness = lambda K0: K0
```

em `ross/multi_rotor/multi_rotor.py:253-254`. Logo:

\[
\mathbf K_{\mathrm{ROSS}}=\mathbf K_0,
\]

e a interação de malha é introduzida somente pela força não linear de backlash.

**Classificação:** fato de código, com confiança alta.

## 3. Escopo e limitações

Esta auditoria reconstrói o comportamento que resulta das versões atuais dos arquivos. Ela não prova qual versão histórica do ROSS foi usada para gerar figuras antigas da dissertação. É possível que uma versão anterior utilizasse diretamente o atributo `gear_mesh_stiffness`; essa possibilidade não pode ser confirmada pelos arquivos analisados.

Também não foram encontrados resultados numéricos de bifurcação gerados pelos scripts dentro de `mestrado`; há apenas dados de referência do artigo em `mestrado/cvs_paper`. Assim, não foi possível fazer rastreamento ponto a ponto entre uma curva salva e uma configuração exata.

## 4. Arquitetura da implementação da dissertação

A classe da dissertação faz uma cópia profunda do `MultiRotor` recebido (`mestrado/backlash.py:619-668`), monta as matrizes lineares do sistema e adiciona uma força de malha não linear durante um integrador Newmark próprio.

A sequência lógica é:

1. criar o `MultiRotor` convencional;
2. tentar remover a rigidez linear da malha por atributos posteriores à construção;
3. calcular deslocamento de transmissão, folga dinâmica, rigidez variável, amortecimento e força de contato;
4. transformar a força escalar de contato em forças generalizadas;
5. resolver o equilíbrio não linear iterativamente em cada passo de tempo.

As matrizes usadas no integrador são obtidas em `mestrado/backlash.py:1053-1059`:

\[
\mathbf M=\mathbf M(q),\qquad
\mathbf C=\mathbf C(\Omega)+\Omega\mathbf G,
\qquad
\mathbf K=\mathbf K(\Omega).
\]

Na prática, `M` e `K` são usadas como matrizes fixas no passo, enquanto a não linearidade é transportada pela força de backlash.

## 5. Arquitetura da implementação atual do ROSS

O ROSS atual centraliza a geometria da malha, a rigidez e o objeto `Backlash` em `Mesh`. A criação ocorre em `ross/multi_rotor/mesh.py:121-232`; o objeto de backlash é construído em `ross/multi_rotor/mesh.py:210-229`.

O `MultiRotor` cria a matriz geométrica de acoplamento em `ross/multi_rotor/multi_rotor.py:239-251` e escolhe, ainda no construtor, se essa matriz será adicionada ao sistema linear (`ross/multi_rotor/multi_rotor.py:253-256`). Na integração, a força nativa de backlash é conectada ao vetor de forças em `ross/multi_rotor/multi_rotor.py:963-1014`.

Essa arquitetura evita o acoplamento linear quando a malha não linear está ativa, desde que seja usado o objeto nativo `mesh.backlash`.

## 6. Cinemática do deslocamento de transmissão

Na implementação da dissertação, o deslocamento de transmissão dinâmico, antes da aplicação da folga, é calculado em `mestrado/backlash.py:256-260`. Em forma compacta:

\[
\begin{aligned}
\delta ={}&
\big[(x_1-x_2)\sin\psi+(y_1-y_2)\cos\psi
+R_1\theta_1+R_2\theta_2\big]\cos\beta_h\\
&+\big[-z_1+z_2
+(R_1\varphi_{x1}+R_2\varphi_{x2})\sin\psi
+(R_1\varphi_{y1}+R_2\varphi_{y2})\cos\psi\big]\sin\beta_h
-e(t).
\end{aligned}
\]

O núcleo atual do ROSS reconstrói as mesmas posições instantâneas, ângulos e projeções em `ross/multi_rotor/mesh.py:1098-1181`.

**Conclusão:** não foi identificada física adicional no deslocamento de transmissão da dissertação. As duas implementações representam os mesmos termos translacionais, rotacionais, torsionais, erro de transmissão e ângulo de hélice.

**Classificação:** fato de código, confiança alta.

## 7. Folga dinâmica

A dissertação calcula a folga efetiva em `mestrado/backlash.py:265-268`:

\[
b_t=b_0+(R_1+R_2)
\left[\operatorname{inv}(\alpha)-\operatorname{inv}(\alpha_0)\right]
\cos\beta_h,
\]

onde:

\[
\operatorname{inv}(\alpha)=\tan\alpha-\alpha.
\]

A função involuta do ROSS está em `ross/multi_rotor/utils.py:13-28`, e o mesmo cálculo é realizado no núcleo de backlash em `ross/multi_rotor/mesh.py:1101-1145`.

**Conclusão:** as expressões são equivalentes. Diferenças numéricas nessa parcela devem vir dos estados fornecidos à função, de parâmetros ou do integrador, não da fórmula da folga.

## 8. Lei de contato rígida e suavizada

As funções por partes para contato no flanco positivo, ausência de contato e contato no flanco negativo aparecem em `mestrado/backlash.py:311-375`. O ROSS atual contém as mesmas formas em `ross/multi_rotor/mesh.py:844-970`.

Para a versão rígida, a função é equivalente a:

\[
f(\delta,b)=
\begin{cases}
\delta-b,&\delta>b,\\
0,&|\delta|\le b,\\
\delta+b,&\delta<-b.
\end{cases}
\]

A derivada temporal utilizada no amortecimento segue a região de contato selecionada. A versão suavizada também preserva a estrutura matemática entre as implementações.

**Conclusão:** a lei local de abertura e fechamento do contato não explica, isoladamente, a divergência observada.

## 9. Rigidez variável da malha

Na dissertação, a rigidez instantânea é obtida por interpolação bilinear de uma tabela bidimensional em `mestrado/backlash.py:139-170` e usada em `mestrado/backlash.py:383-386`. A interpolação atual está em `ross/multi_rotor/utils.py:52-87` e é algebraicamente equivalente, inclusive no tratamento periódico do ângulo.

Entretanto, a **construção da tabela** pode ser diferente:

- a dissertação aceita diretamente níveis `kd` e `ks` e cria um perfil quadrado em `mestrado/backlash.py:971-992`;
- o ROSS atual cria o perfil quadrado por série de Fourier, com média `Kg=self.stiffness` e amplitude relacionada a `Ksq_ratio`, em `ross/multi_rotor/mesh.py:330-409`;
- quando não é fornecida rigidez, o ROSS calcula uma rigidez média equivalente em `ross/multi_rotor/mesh.py:167-180`.

No caso de validação, os níveis fornecidos são aproximadamente:

\[
k_d=6{,}5072\times10^8\ \mathrm{N/m},\qquad
k_s=3{,}6228\times10^8\ \mathrm{N/m}.
\]

A média aritmética é aproximadamente (5{,}065\times10^8\ \mathrm{N/m}), e a razão entre a meia amplitude e a média é aproximadamente 0,285. Isso é próximo, mas não idêntico, ao `Ksq_ratio=0.275` configurado em `mestrado/bifurcation_validation.py:41-45`. Além disso, a média do ROSS é calculada independentemente da dupla `kd/ks`.

**Consequência matemática:** níveis, fase, suavidade da transição e conteúdo harmônico da excitação paramétrica podem mudar.

**Hipótese causal:** essa diferença pode alterar a localização ou intensidade de picos e regiões de contato intermitente. Confiança causal média.

## 10. Amortecimento de malha

A força escalar de contato é calculada na dissertação em `mestrado/backlash.py:383-386`:

\[
c_m=2\zeta\sqrt{k_m M_{eq}},
\qquad
F_m=k_m f(\delta,b)+c_m\dot f(\delta,b).
\]

O ROSS atual usa a mesma estrutura em `ross/multi_rotor/mesh.py:1207-1215`. A massa equivalente também é calculada de forma correspondente em `mestrado/backlash.py:651` e `ross/multi_rotor/mesh.py:686-690`.

**Conclusão:** a fórmula de amortecimento não apresenta divergência estrutural. Ela ainda pode produzir valores diferentes se `k_m` vier de tabelas distintas.

## 11. Transformação para forças generalizadas

A dissertação deriva o deslocamento de transmissão em relação aos seis graus de liberdade de cada engrenagem em `mestrado/backlash.py:270-308`, e aplica:

\[
Q_i=-F_m\frac{\partial\delta}{\partial q_i}
\]

em `mestrado/backlash.py:389-403`.

O ROSS atual realiza a mesma transformação em `ross/multi_rotor/mesh.py:1147-1181` e `ross/multi_rotor/mesh.py:1207-1215`; o vetor final é montado em `ross/multi_rotor/mesh.py:804-807`.

**Conclusão:** sinais e projeções essenciais são equivalentes. Não foi encontrada força generalizada extra exclusiva da dissertação.

## 12. Graus de liberdade ativos

Ambas as formulações admitem seis graus de liberdade por engrenagem. No caso validado, porém, `helix_angle=0`, conforme `mestrado/bifurcation_validation.py:11-45`. Com β_h = 0:

- os termos axiais (z) desaparecem do deslocamento de transmissão;
- os termos de inclinação φ_x e φ_y também desaparecem;
- permanecem diretamente ativos (x), (y) e a rotação torsional de cada engrenagem.

A dissertação ainda inclui os 12 índices das duas engrenagens na construção numérica da tangente em `mestrado/backlash.py:1078-1081`, mas, para engrenagens cilíndricas de dentes retos, seis dessas derivadas de força devem ser nulas ou numericamente desprezíveis.

**Conclusão:** o uso formal de 12 graus de liberdade não representa física adicional no caso de validação.

## 13. Matriz linear de acoplamento

O ROSS constrói uma matriz de acoplamento 12×12 em `ross/multi_rotor/multi_rotor.py:588-761`, usando ângulo de pressão, ângulo de hélice, orientação e raios primitivos. A matriz dimensional é obtida pela multiplicação por `self.mesh.stiffness` em `ross/multi_rotor/multi_rotor.py:763-782`.

Para engrenagens de dentes retos e orientação nominal fixa, essa matriz representa essencialmente um produto externo de posto um associado ao gradiente nominal do deslocamento ao longo da linha de ação:

\[
\mathbf K_m\approx k_m\mathbf g_0\mathbf g_0^T.
\]

Os termos torsionais usam (R_p\cos\alpha_0=R_b). Portanto, a matriz liga deslocamentos (x), (y) e torções das duas engrenagens ao longo da linha de ação nominal.

Ela não é uma rigidez perpendicular independente; contudo, por alterar os modos globais, pode modificar também amplitudes observadas em direções cartesianas.

## 14. Por que `gear_mesh_stiffness = 0` não remove o acoplamento atual

O ponto decisivo é a diferença entre um atributo criado posteriormente e o atributo realmente consumido pela montagem:

- atribuição da dissertação: `mestrado/backlash.py:665-667`;
- rigidez armazenada em `Mesh`: `ross/multi_rotor/mesh.py:121-232`;
- multiplicação efetiva: `self.K_coupling * self.mesh.stiffness` em `ross/multi_rotor/multi_rotor.py:780`;
- inclusão na matriz global: `ross/multi_rotor/multi_rotor.py:784-821`.

Não há, no caminho atual de montagem, leitura de `self.multirotor.gear_mesh_stiffness` depois que a instância já foi criada. A atribuição cria ou altera um atributo sem efeito sobre `self.mesh.stiffness`.

Além disso, `update_mesh_stiffness=False` apenas impede a atualização temporal da rigidez; não substitui `self.add_coupling_stiffness`, nem zera a rigidez já armazenada.

**Conclusão:** o acoplamento linear permanece integralmente presente no código da dissertação quando ele é executado contra o ROSS atual.

## 15. Comparação formal: zeramento versus função lambda

As duas estratégias seriam equivalentes apenas se o zeramento atingisse exatamente o escalar usado na montagem:

\[
k_m=0
\quad\Longrightarrow\quad
\mathbf K_m=k_m\mathbf K_c=\mathbf 0.
\]

Isso poderia ser obtido, em princípio, zerando `self.multirotor.mesh.stiffness` antes da montagem ou substituindo a função de inclusão. O código analisado não faz nenhuma dessas duas operações.

No ROSS nativo, a função lambda não zera o valor físico armazenado em `Mesh`; ela impede que a parcela seja somada à matriz linear. Isso permite que a tabela de rigidez continue disponível para a força não linear, sem duplicar o caminho de força.

Assim:

- **ROSS nativo:** rigidez de malha disponível para (F_m), ausente de μK linear;
- **dissertação contra o ROSS atual:** rigidez de malha disponível para (F_m) e também presente em μK linear.

**Resposta direta à questão central:** não, as estratégias não são equivalentes na versão atual.

## 16. Consequência do duplo caminho de rigidez

Com o acoplamento residual, a equação resolvida pelo código da dissertação torna-se:

\[
\mathbf M\ddot{\mathbf q}
+\mathbf C\dot{\mathbf q}
+\left(\mathbf K_0+\mathbf K_m\right)\mathbf q
=\mathbf F_{ext}+\mathbf F_{backlash}(\mathbf q,\dot{\mathbf q},t).
\]

No ROSS nativo com backlash:

\[
\mathbf M\ddot{\mathbf q}
+\mathbf C\dot{\mathbf q}
+\mathbf K_0\mathbf q
=\mathbf F_{ext}+\mathbf F_{backlash}(\mathbf q,\dot{\mathbf q},t).
\]

Isso produz três consequências:

1. **dentro da folga:** o modelo da dissertação ainda transmite força pela matriz linear, embora a lei não linear indique ausência de contato;
2. **durante o contato:** a rigidez linear e a rigidez não linear atuam simultaneamente;
3. **globalmente:** frequências naturais, formas modais e fases de resposta são alteradas antes mesmo de considerar impactos.

Esse é o desvio estrutural mais forte encontrado na auditoria.

## 17. Integrador Newmark da dissertação

O preditor aparece em `mestrado/backlash.py:415-419`. A cada iteração de Newton–Raphson, a força não linear é recalculada com o estado candidato e incluída no resíduo em `mestrado/backlash.py:422-439`:

\[
\mathbf r=mathbf F_{ext}+\mathbf F_{backlash}
-\left(\mathbf M\ddot{\mathbf q}+\mathbf C\dot{\mathbf q}+\mathbf K\mathbf q\right).
\]

A matriz efetiva inclui:

\[
\mathbf J_{eff}
=\mathbf M+\gamma\Delta t\,\mathbf C
+\beta\Delta t^2\mathbf K
-\beta\Delta t^2\frac{\partial\mathbf F_{backlash}}{\partial\mathbf q},
\]

com a tangente da força não linear aproximada por diferenças finitas em `mestrado/backlash.py:442-479`. A tangente é reconstruída nas iterações (`mestrado/backlash.py:488-533`).

O algoritmo também usa subpassos adaptativos, interpola força externa, ângulo e erro de transmissão e reduz o passo em caso de falha, conforme `mestrado/backlash.py:536-612`.

**Consequência:** contato, perda de contato e reversão de flanco são tratados implicitamente dentro do equilíbrio do passo, com refinamento local.

## 18. Integrador Newmark atual do ROSS

O `newmark` atual define por padrão `newmark_type="simple"`, γ = 0,5, β = 0,25 e tolerância (10^{-6}), em `ross/utils.py:734-844`.

No caminho simples, a função do sistema é chamada uma vez para o passo usando o estado anterior, em `ross/utils.py:917-935`. O vetor do lado direito é então mantido durante o laço de convergência de Newmark em `ross/utils.py:849-900`.

Quando a força de backlash é fornecida por `system_func`, isso significa que, no modo simples padrão, ela não é recalculada com cada estado candidato de Newton dentro do mesmo passo. Em termos práticos, a força não linear fica defasada ou explicitada no macro-passo.

**Consequência matemática:** o algoritmo atual padrão não é equivalente ao Newmark interno da dissertação, apesar de ambos usarem γ = 0,5 e β = 0,25.

Uma configuração explícita com outro tipo de Newmark poderia alterar essa conclusão, mas os códigos analisados não demonstram o uso dessa alternativa.

## 19. Comparação dos tratamentos numéricos

| Aspecto | Dissertação | ROSS atual, padrão simples | Efeito provável |
|---|---|---|---|
| Força de backlash no passo | Recalculada em cada iteração | Avaliada a partir do estado de entrada do passo | Diferença de fase e de transição de contato |
| Tangente não linear | Diferenças finitas | Não incorporada da mesma forma no caminho simples | Convergência e trajetória distintas |
| Passo adaptativo | Sim, com subdivisão local | Não equivalente no caminho observado | Impactos podem ser resolvidos em instantes diferentes |
| Falha de convergência | Reduz o subpasso | Estratégia distinta | Diferente robustez perto de descontinuidades |
| Variáveis interpoladas no subpasso | Força, ângulo e erro | Caminho próprio do integrador | Fase da excitação pode mudar |

**Hipótese causal:** em uma faixa sensível a perda e recuperação de contato, essas diferenças podem deslocar bifurcações, suprimir ou introduzir irregularidade e mudar a densidade do mapa de Poincaré. Confiança causal média a alta, mas sem validação dinâmica nesta auditoria.

## 20. Configuração da varredura de bifurcação

O script define engrenagens de 20 dentes, módulo 0,01 m, ângulo de pressão de 20°, dentes retos, massas de 6,57 kg, inércias de 0,0365 kg·m², mancais de (10^8\ \mathrm{N/m}) e amortecimento de 512,64 N·s/m em `mestrado/bifurcation_validation.py:11-45`.

São usados:

- folga nominal de 50 μm;
- erro de transmissão de 20 μm;
- torque com componente média de 300 N·m e harmônica de 100 N·m;
- rigidez `kd/ks` explícita;
- 6000 pontos por revolução;
- `use_coupling=False`.

Esses parâmetros aparecem em `mestrado/bifurcation_validation.py:53-105`.

A classe é configurada com 30 ciclos retidos e 15 descartados (`mestrado/bifurcation_validation.py:63-66`). Pela fórmula de tempo em `mestrado/backlash.py:653-658`, esses “ciclos” são revoluções do eixo, não ciclos de engrenamento. Para 20 dentes, os 30 ciclos retidos correspondem a aproximadamente 600 períodos de malha.

O estado final de uma velocidade não é passado para a seguinte: a captura dos estados finais está comentada em `mestrado/bifurcation_validation.py:105-109`. Cada velocidade reinicia a partir do estado inicial nulo.

**Consequência:** o diagrama não é uma continuação de ramo. Reinicializações podem selecionar atratores diferentes e produzir saltos ou picos que dependem do transitório descartado.

## 21. Grandeza mostrada no diagrama de bifurcação

O mapa de Poincaré é amostrado no período de engrenamento em `mestrado/bifurcation_validation.py:113-120`. A grandeza armazenada é o deslocamento (x_1) do centro da engrenagem motora, convertido para micrômetros, em `mestrado/bifurcation_validation.py:122-133`.

Logo, o segundo pico observado não é diretamente um pico de DTE, força de contato ou deslocamento ao longo da linha de ação. Ele é um pico de uma coordenada cartesiana do rotor.

Movimento paralelo à linha de ação altera diretamente a penetração e a força. Movimento perpendicular não entra na penetração nominal de primeira ordem, mas pode alterar distância entre centros, β, ângulo de pressão instantâneo, razão de contato e a resposta modal global.

**Conclusão:** a decomposição paralelo/perpendicular ajuda a interpretar a trajetória, mas o simples aumento de (x_1) não identifica sozinho a origem física do segundo pico.

## 22. Análise do segundo pico

### Fato estabelecido

O modelo da dissertação, executado contra o ROSS atual, preserva a matriz linear completa de acoplamento e ainda adiciona a força não linear. Isso altera diretamente a matriz modal que governa (x_1).

### Consequência matemática

A rigidez adicional pode:

- deslocar frequências naturais e velocidades de ressonância;
- modificar formas modais e a participação de (x_1);
- transmitir força mesmo durante a região morta do backlash;
- somar rigidez durante contato e mudar a condição de perda de contato.

### Hipótese causal principal

O acoplamento linear residual é o candidato estrutural mais forte para explicar o segundo pico de (x_1). A confiança é **alta** quanto à existência da diferença e **média** quanto à atribuição causal do pico, porque não houve reprodução numérica controlada.

### Hipóteses secundárias

1. O perfil `kd/ks` da dissertação não é idêntico ao perfil de Fourier/média equivalente do ROSS; isso muda harmônicos da rigidez.
2. O Newmark interno pode seguir um ramo dinâmico diferente do Newmark simples atual.
3. O reinício em estado nulo em cada velocidade pode selecionar atratores diferentes e destacar um pico local.
4. Um arquivo de cache de rigidez desatualizado pode introduzir uma tabela incompatível com os parâmetros aparentes.

## 23. Análise da irregularidade próxima de 4500 rpm

O script de validação do artigo contém uma execução específica a 4500 rpm e analisa (x_1), deslocamento de transmissão, força de malha, espectro e mapa de Poincaré em `mestrado/validation_paper_01.py:366-384`. A configuração do modelo e a chamada do Newmark interno aparecem em `mestrado/validation_paper_01.py:179-265`.

### Fatos relevantes

- o contato é unilateral e pode alternar entre abertura, contato positivo e contato negativo;
- a rigidez varia periodicamente;
- o acoplamento linear residual permanece ativo no caminho da dissertação atual;
- o integrador da dissertação recalcula a força e sua tangente dentro da iteração;
- o ROSS padrão simples trata a força de modo diferente dentro do passo.

### Interpretação

Regiões de contato intermitente são altamente sensíveis a pequenas mudanças de fase, rigidez efetiva, passo temporal e condição inicial. A dupla rigidez pode deslocar o limiar de perda de contato e a vizinhança de ressonância; o integrador pode então resolver uma sequência diferente de impactos e recontatos.

### Hipótese causal

A explicação mais plausível, pela análise estática, é uma combinação de:

1. alteração estrutural de modos e contato pelo acoplamento linear residual;
2. diferença no tratamento implícito da força de backlash;
3. diferença no conteúdo harmônico da rigidez variável.

Não é possível afirmar estaticamente se a irregularidade a 4500 rpm é caos físico, quase-periodicidade, um transitório ainda não eliminado ou artefato numérico. Essa classificação exigiria séries temporais e testes de convergência, que estão fora do escopo solicitado.

## 24. Cache da tabela de rigidez

A dissertação salva e reutiliza uma tabela `.npz` em `mestrado/backlash.py:947-1004`. O nome do cache depende essencialmente do arquivo chamador (`mestrado/backlash.py:950-967`) e não codifica ou valida todos os parâmetros que determinam a tabela, como geometria, `kd`, `ks`, número de pontos ou modo de rigidez.

Nos scripts de validação, a função de criação da tabela é chamada antes da integração, mas o retorno não é necessariamente usado diretamente; a integração pode recarregar o cache. Isso cria duas possibilidades:

- se o cache acabou de ser criado com os parâmetros corretos, o resultado é consistente;
- se já existia um cache com o mesmo nome e parâmetros antigos, a execução pode usar silenciosamente uma tabela incompatível.

**Classificação:** risco de reprodutibilidade confirmado por inspeção; efeito efetivo nos resultados históricos indeterminado.

## 25. Incompatibilidades de API com o ROSS atual

Foram encontradas duas incompatibilidades adicionais:

1. `mestrado/backlash.py:994-998` chama `self.multirotor.mesh.get_variable_stiffness`, enquanto a API atual expõe `get_variable_equivalent_stiffness`. Esse ramo pode falhar quando usado com a versão atual.
2. `mestrado/backlash.py:1021` e `mestrado/backlash.py:1087` acessam `self.multirotor.orientation_angle`. No ROSS atual, a orientação pertence a `self.mesh.orientation_angle`.

Essas diferenças indicam que `mestrado/backlash.py` foi escrito para outra revisão da API. Elas podem impedir a execução atual ou exigir adaptação, mas não demonstram por si sós a causa de resultados históricos produzidos com uma versão compatível.

## 26. A implementação da dissertação adiciona física ausente no ROSS?

No núcleo físico do backlash, não. A comparação mostra equivalência entre:

- deslocamento de transmissão tridimensional;
- folga dependente da geometria instantânea;
- contato rígido e suavizado;
- amortecimento proporcional à rigidez instantânea e à massa equivalente;
- transformação da força escalar em forças generalizadas;
- interpolação bilinear da rigidez.

A implementação da dissertação parece ser uma versão ancestral ou paralela da formulação hoje encapsulada em `Mesh.Backlash`, e não uma extensão física mais completa.

As diferenças materiais estão na arquitetura de acoplamento, na geração da tabela de rigidez, no cache e no integrador. Para o caso de dentes retos, os graus de liberdade adicionais formais também não introduzem termos físicos ativos além de (x), (y) e torção.

## 27. Tabela consolidada de divergências

| ID | Divergência | Evidência | Tipo | Impacto potencial | Confiança |
|---|---|---|---|---|---|
| D1 | `gear_mesh_stiffness=0` não atinge `mesh.stiffness` | `mestrado/backlash.py:665-667`; `ross/multi_rotor/multi_rotor.py:763-782` | Estrutural | Mantém a matriz linear de malha | Alta |
| D2 | O ROSS nativo usa lambda para não somar a malha linear | `ross/multi_rotor/multi_rotor.py:253-256` | Estrutural | Evita duplicação da interação | Alta |
| D3 | Dissertação atual combina malha linear e força não linear | D1 + `mestrado/backlash.py:422-439` | Estrutural | Modos, ressonâncias, força dentro da folga e contato alterados | Alta |
| D4 | Força local de backlash é matematicamente equivalente | `mestrado/backlash.py:206-410`; `ross/multi_rotor/mesh.py:974-1217` | Equivalência | Não explica sozinha as divergências | Alta |
| D5 | Perfil quadrado `kd/ks` difere do perfil Fourier/`Ksq_ratio` | `mestrado/backlash.py:971-992`; `ross/multi_rotor/mesh.py:330-409` | Paramétrica | Harmônicos, média e fase da rigidez | Alta |
| D6 | Cache não valida parâmetros físicos | `mestrado/backlash.py:947-1004` | Reprodutibilidade | Possível tabela obsoleta | Alta para o risco; indeterminada para ocorrência histórica |
| D7 | Newmark da dissertação recalcula força e tangente | `mestrado/backlash.py:422-612` | Numérica | Tratamento mais fortemente acoplado do contato | Alta |
| D8 | Newmark simples atual mantém RHS no laço interno | `ross/utils.py:849-935` | Numérica | Força não linear defasada no macro-passo | Alta |
| D9 | Varredura reinicia cada velocidade | `mestrado/bifurcation_validation.py:74-109` | Procedimental | Seleção de atrator e saltos dependentes da inicialização | Alta |
| D10 | Grandeza bifurcada é (x_1), não DTE | `mestrado/bifurcation_validation.py:113-133` | Interpretação | Pico não identifica diretamente contato/LOA | Alta |
| D11 | “Ciclos” são revoluções do eixo | `mestrado/backlash.py:653-658`; `mestrado/bifurcation_validation.py:63-66` | Procedimental | Quantidade efetiva de períodos de malha diferente da nomenclatura | Alta |
| D12 | APIs antigas de rigidez e orientação | `mestrado/backlash.py:994-998,1021,1087` | Compatibilidade | Possível falha contra o ROSS atual | Alta |
| D13 | Para dente reto, z/rotações laterais não entram diretamente na DTE | Fórmula em `mestrado/backlash.py:256-308` com β_h = 0 | Física | Reduz os DOFs físicos ativos da malha | Alta |
| D14 | Não há resultados gerados disponíveis para replay estático | inventário de `mestrado`; apenas `cvs_paper` | Evidência | Impede atribuição causal definitiva | Alta |

## 28. Ranking das causas prováveis

### Segundo pico do diagrama de bifurcação

1. **Acoplamento linear residual somado à força não linear** — principal candidato.
2. **Diferença na tabela de rigidez variável** — candidato relevante.
3. **Diferença entre os integradores** — candidato relevante, sobretudo perto de impactos.
4. **Reinício do estado em cada velocidade** — pode mudar o ramo observado.
5. **Cache desatualizado** — possível, sem evidência de que ocorreu na geração histórica.

### Irregularidade próxima de 4500 rpm

1. **Mudança do limiar de perda/reentrada de contato pela dupla rigidez**.
2. **Tratamento iterativo/adaptativo distinto da força de backlash**.
3. **Conteúdo harmônico diferente da rigidez variável**.
4. **Transiente ou seleção de atrator decorrente da inicialização**.
5. **Artefato de cache ou compatibilidade de versão**.

Esse ranking expressa plausibilidade estrutural, não comprovação experimental.

## 29. Respostas objetivas às perguntas centrais

### Zerar a rigidez no código antigo é equivalente à lambda do ROSS?

**Não, não com o ROSS atual.** O código antigo zera um atributo que não é usado na montagem atual, enquanto a lambda impede diretamente a soma da matriz de acoplamento.

### Existe acoplamento residual?

**Sim.** Não é apenas um resíduo pequeno: é a matriz linear completa `K_coupling * mesh.stiffness`, salvo se algum código externo não analisado modificar `mesh.stiffness` ou a função de montagem posteriormente.

### O acoplamento pode explicar o segundo pico?

**Pode e é o candidato principal**, porque altera diretamente os modos e a coordenada (x_1) usada no diagrama. A causalidade definitiva exigiria comparação controlada.

### O movimento perpendicular à linha de ação é a causa?

**Não há evidência estática suficiente.** Ele pode participar por geometria instantânea e acoplamento modal, mas o pico de (x_1) não deve ser interpretado automaticamente como penetração na malha.

### O `backlash.py` contém mais física que o ROSS atual?

**Não no núcleo da força.** As diferenças principais são montagem, rigidez variável, cache e integração temporal.

### A irregularidade de 4500 rpm é necessariamente física?

**Não é possível concluir.** Ela pode ser uma resposta não linear real, um atrator dependente de condição inicial, um efeito do integrador ou uma combinação desses fatores.

## 30. Conclusão

A análise estática identifica uma divergência central e inequívoca: o mecanismo usado pela implementação da dissertação para desativar a rigidez linear não funciona contra a arquitetura atual do ROSS. Como resultado, o modelo mantém a malha linear bilateral e adiciona a força não linear de backlash. O ROSS nativo, por sua vez, remove a matriz linear por construção e deixa a interação exclusivamente na força não linear.

Essa duplicação é a explicação estrutural mais forte para diferenças em picos, frequências, amplitudes e transições de contato, incluindo o segundo pico do diagrama. Próximo de 4500 rpm, seu efeito se combina com um tratamento Newmark substancialmente diferente e com possíveis diferenças na tabela de rigidez, tornando a região especialmente sensível.

Ao mesmo tempo, o núcleo matemático da força de backlash é essencialmente equivalente nas duas implementações. Portanto, a investigação futura deve priorizar montagem e integração, não uma suposta ausência de termos físicos no `Backlash` atual do ROSS.

Qualquer confirmação causal exigiria, em trabalho posterior e explicitamente autorizado, uma sequência controlada em que apenas uma diferença fosse alterada por vez: primeiro o acoplamento linear, depois o integrador, depois a tabela de rigidez e, por fim, o procedimento de continuação/inicialização.
