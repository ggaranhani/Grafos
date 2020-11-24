import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
G = nx.Graph()


# G.add_nodes_from([
#         "Morena Baccarin", 
#         "Sonia Braga", 
#         "Débora Nascimento", 
#         "Wagner Moura", 
#         "Rodrigo Santoro", 
#         "Alice Braga",
#         "Selton Mello",
#         "Bruno Garcia",
#         "Fernanda Montenegro",
#         "Marco Nanini",
#         "Seu Jorge",
#         "Matheus Nachtergaele",
#         "Charles Paraventi",
#         "Fábio Assunção",
#         "Priscila Fantin",
#         "Ana Paula Arósio",
#         "Lima Duarte",
#         "Pedro Cardoso",
#         "Fernanda Torres",
#         "Marieta Severo",
#         "Tonico Pereira",
#         "Lázaro Ramos",
#         "Clint Eastwood",
#         "Charlie Sheen"
#     ]
# )
G.add_nodes_from([
        "Morena Baccarin", 
        "Sonia Braga", 
        "Débora Nascimento", 
        "Wagner Moura", 
        "Rodrigo Santoro", 
        "Alice Braga",
        "Selton Mello",
        "Bruno Garcia",
        "Fernanda Montenegro",
        "Marco Nanini",
        "Seu Jorge",
        "Matheus Nachtergaele",
        "Charles Paraventi",
        "Fábio Assunção",
        "Priscila Fantin",
        "Ana Paula Arósio",
        "Lima Duarte",
        "Pedro Cardoso",
        "Fernanda Torres",
        "Marieta Severo",
        "Tonico Pereira",
        "Lázaro Ramos"
    ]
)

G.add_edge("Wagner Moura", "Alice Braga")
G.add_edge("Wagner Moura", "Selton Mello")
G.add_edge("Wagner Moura", "Rodrigo Santoro")
G.add_edge("Wagner Moura", "Seu Jorge")
G.add_edge("Wagner Moura", "Marco Nanini")
G.add_edge("Wagner Moura", "Lázaro Ramos")
G.add_edge("Wagner Moura", "Pedro Cardoso")
G.add_edge("Wagner Moura", "Lima Duarte")
G.add_edge("Wagner Moura", "Fábio Assunção")
G.add_edge("Fernanda Torres", "Matheus Nachtergaele")
G.add_edge("Fernanda Torres", "Seu Jorge")
G.add_edge("Fernanda Torres", "Selton Mello")
G.add_edge("Fernanda Torres", "Fernanda Montenegro")
G.add_edge("Fernanda Torres", "Selton Mello")
G.add_edge("Fernanda Torres", "Tonico Pereira")
G.add_edge("Fernanda Torres", "Lázaro Ramos")
G.add_edge("Fernanda Torres", "Bruno Garcia")
G.add_edge("Fernanda Torres", "Priscila Fantin")
G.add_edge("Fernanda Torres", "Fábio Assunção")
G.add_edge("Matheus Nachtergaele", "Selton Mello")
G.add_edge("Matheus Nachtergaele", "Marco Nanini")
G.add_edge("Matheus Nachtergaele", "Alice Braga")
G.add_edge("Matheus Nachtergaele", "Seu Jorge")
G.add_edge("Matheus Nachtergaele", "Fernanda Montenegro")
G.add_edge("Matheus Nachtergaele", "Lima Duarte")
G.add_edge("Matheus Nachtergaele", "Bruno Garcia")
G.add_edge("Matheus Nachtergaele", "Pedro Cardoso")
G.add_edge("Matheus Nachtergaele", "Ana Paula Arósio")
G.add_edge("Marco Nanini", "Selton Mello")
G.add_edge("Marco Nanini", "Pedro Cardoso")
G.add_edge("Marco Nanini", "Marieta Severo")
G.add_edge("Marco Nanini", "Fernanda Montenegro")
G.add_edge("Marco Nanini", "Bruno Garcia")
G.add_edge("Marco Nanini", "Fernanda Torres")
G.add_edge("Marco Nanini", "Sonia Braga")
G.add_edge("Marco Nanini", "Débora Nascimento")
G.add_edge("Marco Nanini", "Lima Duarte")
G.add_edge("Charles Paraventi", "Seu Jorge")
G.add_edge("Charles Paraventi", "Wagner Moura")
G.add_edge("Charles Paraventi", "Fábio Assunção")
G.add_edge("Charles Paraventi", "Alice Braga")
G.add_edge("Charles Paraventi", "Priscila Fantin")
G.add_edge("Charles Paraventi", "Ana Paula Arósio")
G.add_edge("Selton Mello", "Morena Baccarin")
G.add_edge("Selton Mello", "Bruno Garcia")
G.add_edge("Selton Mello", "Tonico Pereira")
G.add_edge("Selton Mello", "Fernanda Montenegro")
G.add_edge("Selton Mello", "Alice Braga")
G.add_edge("Selton Mello", "Rodrigo Santoro")
G.add_edge("Selton Mello", "Lima Duarte")
G.add_edge("Pedro Cardoso", "Fernanda Montenegro")
G.add_edge("Pedro Cardoso", "Selton Mello")
G.add_edge("Pedro Cardoso", "Fernanda Torres")
G.add_edge("Pedro Cardoso", "Marieta Severo")
G.add_edge("Pedro Cardoso", "Lázaro Ramos")
G.add_edge("Pedro Cardoso", "Tonico Pereira")
G.add_edge("Pedro Cardoso", "Rodrigo Santoro")
G.add_edge("Pedro Cardoso", "Lázaro Ramos")
# G.add_edge("Sonia Braga", "Clint Eastwood")
# G.add_edge("Sonia Braga", "Charlie Sheen")
# G.add_edge("Clint Eastwood", "Charlie Sheen")
G.add_edge("Débora Nascimento", "Lázaro Ramos")
G.add_edge("Débora Nascimento", "Rodrigo Santoro")
G.add_edge("Débora Nascimento", "Fernanda Montenegro")
G.add_edge("Débora Nascimento", "Wagner Moura")
G.add_edge("Débora Nascimento", "Tonico Pereira")
G.add_edge("Lima Duarte", "Fernanda Montenegro")
G.add_edge("Lima Duarte", "Bruno Garcia")
G.add_edge("Lima Duarte", "Tonico Pereira")
G.add_edge("Ana Paula Arósio", "Rodrigo Santoro")
G.add_edge("Ana Paula Arósio", "Fernanda Montenegro")
G.add_edge("Ana Paula Arósio", "Sonia Braga")
G.add_edge("Ana Paula Arósio", "Tonico Pereira")
G.add_edge("Ana Paula Arósio", "Selton Mello")

# pos = nx.spring_layout(G,k=0.50,iterations=1)
# nx.draw_networkx(G, layout=nx.spring_layout(G), pos=pos)
# plt.savefig("simple_path.png") # save as png
# plt.show() # display

pos = nx.spring_layout(G, k=0.3*1/np.sqrt(len(G.nodes())), iterations=2)
plt.figure(3, figsize=(10, 10))
nx.draw(G, pos=pos)
nx.draw_networkx_labels(G, pos=pos)
plt.savefig("simple_path.png") # save as png
plt.show()


print(nx.shortest_path(G, "Seu Jorge", "Marieta Severo"))
print(nx.shortest_path(G, "Alice Braga", "Fernanda Montenegro"))
print(nx.shortest_path(G, "Lima Duarte", "Charles Paraventi"))
print(nx.shortest_path(G, "Ana Paula Arósio", "Débora Nascimento"))
print(nx.has_path(G, "Ana Paula Arósio", "Débora Nascimento"))

print(nx.cycle_basis(G))
print(nx.cycle_basis(G))