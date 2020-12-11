## Documentation for this module.
## This Python module parses POMDPx files and returns vectors and values that are used for belief update.

import numpy as np

## Documentation for a function.
## This function uses the root of a parsed XML policy file, parses it and returns two lists.
## First list contains policy vectors.
## Second list contains the best actions for each policy.
## @param root Root of XML policy file.
## @returns policy_vectors_field List of policy vectors.
## @returns best_action_list List of best actions.
def import_policy(root):

    for i in root.findall('AlphaVector'):
        policy_vectors_field = []
        best_action_list = []
        observable_states = []
        for m in i.findall('Vector'):
            best_action_list.append(int(m.attrib['action']))
            observable_states.append(int(m.attrib['obsValue']))
            pom = m.text.rstrip(' ').split(' ')
            policy_list = []
            for x in pom:
                policy_list.append(float(x))
                policy_vector = np.array(policy_list)
            policy_vectors_field.append(policy_vector)

    return policy_vectors_field, best_action_list, observable_states


## Documentation for a function.
## This function uses the root of a parsed XML POMDPx file, parses it and returns wanted dictionary
## depending on the input tag.
## @param tag Name of the tag that defines which part of POMDPx wants to be put in dictionary.
## @param root Root of XML POMDPx file.
## @returns dictionary Dictionary that connects wanted matrices with their transitions or observations.
def get_matrix(tag, root, observability):
    for k in root.findall(tag):
        dict_list = []
        count = 0
        for m in k.findall('CondProb'):
            dictionary = {}
            for n in m.findall('Parameter'):
                for o in n.findall('Entry'):
                    key = o.find('Instance').text.split(' ')[0]
                    for p in o.findall('ProbTable'):
                        list1 = []
                        pom2 = p.text.lstrip('\n').rstrip('\n').rstrip(' ').split('\n')
                        pom2 = [x for x in pom2 if x is not '']
                        matrix_height = len(pom2)
                        for x in pom2:
                            pom3 = x.split(' ')
                            matrix_width = 0
                            for y in pom3:
                                if y != '':
                                    matrix_width += 1
                                    list1.append(float(y))
                    vector = np.array(list1).reshape(matrix_height, matrix_width)
                    dictionary[key] = vector
            if tag == 'ObsFunction' and observability[count]:
                dict_list.append(None)
            dict_list.append(dictionary)
            count += 1
    return dict_list


## Documentation for a function.
## This function uses the root of a parsed XML POMDPx file, parses it and returns description od POMDPx
## and basic variables that define POMDP.
## First list contains policy vectors.
## Second list contains the best actions for each policy.
## @param root Root of XML POMDPx file.
## @returns description Description of POMDPx file purpose.
## @returns discount Discount.
## @returns states List of states.
## @returns actions List of actions.
## @returns observations List of possible observations.
def get_general_info(root):
    for child in root:
        if child.tag == 'Description':
            description = child.text
        elif child.tag == 'Discount':
            discount = float(child.text)
    for child in root.findall('Variable'):
        states = []
        actions = []
        observations = []
        state_names = []
        observability = []
        for k in child:
            if k.tag == 'StateVar':
                state_names.append(k.attrib.get('vnamePrev'))
                if k[0].tag == 'ValueEnum':
                    states.append(k[0].text.split(' '))
                else:
                    states.append(["s%s" % x for x in range(1, int(k[0].text) + 1)])
                if k.attrib.get('fullyObs') == 'true':
                    observability.append(True)
                    observations.append(None)
                else:
                    observability.append(False)
            if k.tag == 'ActionVar':
                if k[0].tag == 'ValueEnum':
                    actions.append(k[0].text.split(' '))
                else:
                    actions.append(["a%s" % x for x in range(1, int(k[0].text) + 1)])
            if k.tag == 'ObsVar':
                if k[0].tag == 'ValueEnum':
                    observations.append(k[0].text.split(' '))
                else:
                    observations.append(["o%s" % x for x in range(1, int(k[0].text) + 1)])

    return description, discount, states, actions, observations, state_names, observability

## Documentation for a function.
## This function uses the root of a parsed XML POMDPx file, parses it and returns initial belief over states.
## @param root Root of XML POMDPx file.
## @returns isb_vector Vector that contains initial belief over states
def get_initial_belief(root):

    for k in root.findall('InitialStateBelief'):
        isb_list = []
        for m in k.findall('CondProb'):
            isb = []
            for n in m.findall('Parameter'):
                for o in n.findall('Entry'):
                    for p in o.findall('ProbTable'):
                        pom1 = p.text.split(' ')
                        for x in pom1:
                            isb.append(float(x))
            isb_list.append(isb)

    return isb_list


def get_reward_model(root):
    _, _, states, actions, _, state_names = get_general_info(root)

    reward_model = []
    for i in range(len(actions[0])):
        matrix_array = []
        for state_set in states:
            matrix_array.append(np.zeros(len(state_set)))
        reward_model.append(matrix_array)

    for reward in root.findall('RewardFunction'):
        for function in reward.findall('Func'):
            for parent in function.findall('Parent'):
                state_set_index = state_names.index(parent.text.split(' ')[-1])
            for param in function.findall('Parameter'):
                for entry in param.findall('Entry'):
                    for inst in entry.findall('Instance'):
                        action, state = inst.text.split(' ')
                        action_index = actions[0].index(action)
                        state_index = states[state_set_index].index(state)
                    for val in entry.findall('ValueTable'):
                        reward_model[action_index][state_set_index][state_index]=float(val.text)
    return reward_model, states, actions, state_names
