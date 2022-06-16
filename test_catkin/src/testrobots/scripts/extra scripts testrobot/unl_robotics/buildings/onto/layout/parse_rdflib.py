#!/usr/bin/env python3

import sys
from rdflib import Graph, URIRef, Literal

def main():

    ############
    ## Parse
    #
    print("")
    print("")
    print(" --- Parse --- ")
    print("")
    inputFilename = "simple_model.WorkingWithChanges.ttl"
    model = Graph()
    model.parse(inputFilename)
    print("Parsed file: " + inputFilename)
    print(f"The model has " + str(len(model)) + " statments.")
    print("")

    ###########
    ## Inspect
    #
    print("")
    print(" --- Inspect --- ")
    print("")
    baseURI = "http://linkedbuildingdata.net/ifc/resources20211019_133153/"
    ref = URIRef(baseURI)
    if(ref, None, None) in model:
        print("The base reference is in the model")
    else:
        print("The base reference is **NOT** in the model")
        
    siteRef = URIRef(baseURI + "site_168")
    if(siteRef, None, None) in model:
        print("inst:site_168 is in the model")
    else:
        print("inst:site_168 is **NOT** in the model")
            
    botURI = "https://w3id.org/bot#"
    hasBuildingRef = URIRef(botURI + "hasBuilding")
    if(siteRef, hasBuildingRef, None) in model:
        print("bot:hasBuilding predicate is in the model")
    else:
        print("bot:hasBuilding prediate a is **NOT** in the model")
        
    buildingRef = URIRef(baseURI + "building_145")
    if(siteRef, None, buildingRef) in model:
        print("inst:building_145 is an object in the model")
    else:
        print("inst:building_145 is **NOT** an object in the model")

    if(siteRef, hasBuildingRef, buildingRef) in model:
        print("inst:building_145 is an object with a predicate")
    else:
        print("inst:building_145 is **NOT** an object with a predicate")

    print("")

    ###########
    ## Modify
    #
    print("")
    print(" --- Modify --- ")
    print("")
    print(f"Before modification the model has " + str(len(model)) + " statments.")

    #Loop over 3 columns
    for col in range(3):

        #Add a triple connecting 2 columns as being adjacent elements
        column_obj  = 4520 + col
        column_subj = 4647 + col
        column_obj_ref = URIRef(baseURI + "column_" + str(column_obj))
        column_subj_ref = URIRef(baseURI + "column_" + str(column_subj))
        adjacentRef = URIRef(botURI + "adjacentElement")
        model.add((column_obj_ref, adjacentRef, column_subj_ref))
        print("Added a triple connecting column_" + str(column_obj) + " to column_" + str(column_subj) + " as adjacent elements")

    print(f"After modification the model now has " + str(len(model)) + " statments.")
    print("")

    ############
    ## Serialize
    #
    print("")
    print(" --- Serialize --- ")
    print("")

    #Print to screen
    #print(model.serialize(format="turtle"))
    
    #Save to disk
    outputFilename = "rdflib.ttl"
    model.serialize(outputFilename, format="turtle")
    print("Model contents written to file: " + outputFilename)
    
    print("")
    print(" --- Finished! ---")
    print("")
    
    
# Call main and exit success
if __name__ == "__main__":
    main()
    sys.exit()
