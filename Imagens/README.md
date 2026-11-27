[README.md](https://github.com/user-attachments/files/35527962/README.md)
# Documentação visual — Projeto Robô Delta

Este diretório destina-se ao armazenamento e à organização dos registros fotográficos, ilustrações, diagramas e demais materiais visuais relacionados ao desenvolvimento do protótipo de robô paralelo Delta, concebido como artefato educacional no contexto de um Trabalho de Conclusão de Curso (TCC) da Universidade Estadual de Goiás (UEG).

A documentação visual complementa os códigos-fonte e a documentação técnica do projeto, favorecendo a compreensão de sua arquitetura, de seu funcionamento, das etapas de construção e dos procedimentos de avaliação experimental.

## 1. Objetivos

- Documentar visualmente o processo de desenvolvimento do protótipo.
- Apresentar detalhes construtivos, mecânicos, eletrônicos e computacionais.
- Auxiliar na compreensão da montagem, da integração e da operação do sistema.
- Complementar os registros dos ensaios e das limitações observadas.
- Favorecer a aprendizagem experimental, a reprodutibilidade e o aperfeiçoamento colaborativo do projeto.

## 2. Conteúdo previsto

| Categoria | Descrição |
| --- | --- |
| Protótipo | Fotografias gerais do robô e de sua configuração física. |
| Projeto mecânico | Imagens dos componentes, modelos tridimensionais, dimensões, articulações e etapas de fabricação e montagem. |
| Sistema eletrônico | Fotografias da placa eletrônica, conexões, sensores, atuadores, pinagem e diagramas de integração. |
| Firmware e controle | Diagramas da arquitetura embarcada, dos estados operacionais e do fluxo de comandos. |
| Sistema supervisório | Capturas de tela da interface gráfica, recursos de controle, monitoramento e configurações. |
| Visão computacional | Registros da segmentação de cores, identificação de objetos, calibração por homografia e transformação de coordenadas. |
| Integração e Pick and Place | Imagens explicativas da coleta, do transporte e da deposição de objetos. |
| Validação experimental | Registros dos procedimentos de teste, das posições avaliadas, das ocorrências e dos resultados observados. |

## 3. Organização sugerida

À medida que os materiais forem adicionados, poderão ser organizados nas seguintes subpastas:

```text
Imagens/
├── README.md
├── 01_Prototipo/
├── 02_Projeto_Mecanico/
├── 03_Sistema_Eletronico/
├── 04_Firmware_e_Controle/
├── 05_Sistema_Supervisorio/
├── 06_Visao_Computacional/
├── 07_Integracao_Pick_and_Place/
└── 08_Validacao_Experimental/
```

Essa estrutura é uma sugestão de organização e pode ser adaptada conforme o conteúdo disponibilizado.

## 4. Identificação e contextualização das imagens

Recomenda-se utilizar nomes de arquivos descritivos, sem espaços ou caracteres especiais, para facilitar a localização e o uso das imagens no GitHub.

Exemplos de nomes de arquivos:

- `prototipo_vista_frontal.jpg`
- `estrutura_mecanica_detalhes.png`
- `placa_eletronica_conexoes.jpg`
- `firmware_diagrama_estados.png`
- `supervisorio_interface_principal.png`
- `visao_calibracao_homografia.png`
- `validacao_coerencia_cinematica_eixo_x.png`
- `validacao_pick_and_place_posicao_01.jpg`

Sempre que pertinente, recomenda-se acompanhar a imagem de informações sobre sua autoria, data, versão do protótipo, condições de operação, unidades de medida e relação com o procedimento documentado. Diagramas e esquemas conceituais devem ser identificados como tais.

## 5. Rigor técnico e registros experimentais

Os materiais visuais devem ser interpretados em conjunto com a documentação técnica e os registros experimentais correspondentes.

Fotografias, capturas de tela, esquemas ilustrativos e resultados de medições possuem naturezas distintas. Uma representação conceitual não deve ser interpretada como comprovação experimental de dimensões, precisão, segurança ou desempenho do protótipo.

Em particular, as ilustrações relacionadas à coerência cinemática devem distinguir os deslocamentos medidos ao longo do eixo X das observações qualitativas, não registradas sistematicamente, ao longo do eixo Y. A representação de uma região operacional reduzida ou simétrica deve ser identificada como hipótese ilustrativa quando não sustentada por medições suficientes.

A documentação visual da calibração por homografia deve distinguir a adaptação do mapeamento visual às posições fisicamente alcançadas pelo manipulador da correção do modelo cinemático.

## 6. Finalidade educacional

Além de registrar o desenvolvimento do protótipo, este diretório pretende constituir um recurso complementar para o estudo de robótica paralela, automação, sistemas embarcados, visão computacional e integração de sistemas.

Os materiais poderão auxiliar estudantes, docentes e pesquisadores na compreensão das decisões de projeto, na identificação de limitações e na proposição de melhorias. A disponibilização dos registros não implica certificação de segurança industrial nem comprovação de eficácia pedagógica.

## 7. Autoria, licenciamento e reutilização

A disponibilização dos materiais visuais busca incentivar o estudo, o compartilhamento do conhecimento e a reprodução do projeto.

A licença MIT adotada para os códigos-fonte não deve ser automaticamente interpretada como licença para todos os materiais visuais deste diretório. A licença de cada material deve ser respeitada.

Para imagens, diagramas e ilustrações de autoria própria cujos direitos possam ser licenciados pelo autor, recomenda-se a adoção da licença [Creative Commons Atribuição 4.0 Internacional (CC BY 4.0)](https://creativecommons.org/licenses/by/4.0/deed.pt-br), mediante indicação expressa em arquivo de licença ou no próprio material. Essa licença permite compartilhamento e adaptação, inclusive comercial, com atribuição adequada, indicação de alterações e observância de seus termos.

Materiais de terceiros, marcas institucionais e logotipos permanecem sujeitos aos direitos e às condições de utilização de seus respectivos titulares. A presença desses elementos neste diretório não implica autorização irrestrita de reutilização.

### Atribuição sugerida

> Projeto Robô Delta — Trabalho de Conclusão de Curso, Universidade Estadual de Goiás (UEG). Material de autoria identificada no registro correspondente. Indicar o endereço do repositório, a licença aplicável e eventuais alterações.

---

**Projeto Robô Delta — Robótica educacional, documentação aberta e aprendizagem experimental.**
