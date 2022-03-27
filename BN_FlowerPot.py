
from pgmpy.models import BayesianNetwork
from pgmpy.inference import VariableElimination

flowerPot_model = BayesianNetwork(
    [
        ("Wind", "PotFalls"),
        ("Cat", "PotFalls"),
        ("Cat", "DogBarks"),
        ("PotFalls", "PotBreaks"),
        ("PotBreaks", "OwnerWakes"),
        ("DogBarks", "OwnerWakes"),
    ]
)

from pgmpy.factors.discrete import TabularCPD

cpd_wind = TabularCPD(
    variable="Wind", variable_card=2, values=[[0.5], [0.5]]
)
cpd_cat = TabularCPD(
    variable="Cat", variable_card=2, values=[[0.3], [0.7]]
)
cpd_potfalls = TabularCPD(
    variable="PotFalls",
    variable_card=2,
    values=[[0.999, 0.98, 0.95, 0.815], [0.001, 0.02, 0.05, 0.185]],
    evidence=["Wind", "Cat"],
    evidence_card=[2, 2],
)
cpd_potbreaks = TabularCPD(
    variable="PotBreaks",
    variable_card=2,
    values=[[0.997, 0.1], [0.003, 0.9]],
    evidence=["PotFalls"],
    evidence_card=[2],
)
cpd_dogbarks = TabularCPD(
    variable="DogBarks",
    variable_card=2,
    values=[[0.95, 0.3], [0.05, 0.7]],
    evidence=["Cat"],
    evidence_card=[2],
)
cpd_ownerwakes = TabularCPD(
    variable="OwnerWakes",
    variable_card=2,
    values=[[0.95, 0.7, 0.5, 0.05], [0.05, 0.3, 0.5, 0.95]],
    evidence=["DogBarks", "PotBreaks"],
    evidence_card=[2, 2],
)

flowerPot_model.add_cpds(
    cpd_wind, cpd_cat, cpd_potfalls, cpd_potbreaks, cpd_dogbarks, cpd_ownerwakes
)


print(flowerPot_model.check_model())

print(flowerPot_model.nodes())

print(flowerPot_model.edges())

print(flowerPot_model.local_independencies("Wind"))


from pgmpy.inference import VariableElimination
potfalls_infer = VariableElimination(flowerPot_model)


q = potfalls_infer.query(variables=["OwnerWakes","Cat"], evidence={"Wind": 1})
#q = potfalls_infer.query(variables=["PotFalls", "Cat"], evidence={"Wind": 1})
#q = potfalls_infer.query(variables=["DogBarks","Wind"], evidence={"Cat":0})

print(q)