.. _SCORING:

=======
Scoring
=======

There are 3 major components of the Trial Score.

  1. **Cost Factor**: How much does the system (sensors) cost?
  2. **Efficiency Factor**: How fast or efficiently did the system complete the task(s)
  3. **Task Score**: How well did the task(s) get performed? 

-----------
Cost Factor
-----------

The Cost Factor :math:`CF` compares the cost of the sensors chosen by the team to the average of all sensor configurations across all teams.

  * :math:`TC` is the total cost of the sensors in the team's configuration.
  * :math:`TC_{avg}` is the average sensor cost across all teams.
  * :math:`w_c` is a weighting constant for cost factor.


.. admonition:: Cost Factor
  :class: tip
  :name: cost-factor

  .. math::

    CF = w_c \cdot \frac{TC_{avg}}{TC}

-----------------
Efficiency Factor
-----------------

The Efficiency Factor :math:`EF_i` for order :math:`i` compares the time to complete order :math:`i` for the team to the average of all teams's times to complete order :math:`i`.

  * :math:`T_i` is the time to complete order :math:`i`
  * :math:`T_{avg_{i}}` is the average time to complete order :math:`i` for all teams
  * :math:`w_t` is a weighting constant for efficiency factor.

.. admonition:: Efficiency Factor
  :class: tip
  :name: efficiency-factor

  .. math::

    EF_i = w_t \cdot \frac{T_{avg_{i}}}{T_i}

-----------
Task Scores
-----------

Each submitted order will recieve a task score based on the required task for that order (Kitting, Assembly, or Combined). Each task score is generated from Boolean conditions.

Kitting Task Score
==================

  * A kitting task has :math:`n` parts that need to be placed on the kitting tray.
  
  * A shipment has :math:`m` parts on the kitting tray.
  
  * For each task there are two Boolean conditions:
  
    * :math:`\texttt{isCorrectTrayID} \rightarrow A ~~` : true if the shipment tray ID is correct.
      
    * :math:`\texttt{isCorrectDestination} \rightarrow B ~~` : true if the shipment was sent to the correct destination.
  
  * For each quadrant :math:`q` of the kitting tray there are four Boolean conditions:
  
    * :math:`\texttt{isCorrectType}_{q} \rightarrow C ~~` : true if the part type in quadrant :math:`q` is correct.
    
    * :math:`\texttt{isCorrectColor}_{q} \rightarrow D ~~` : true if the part color in quadrant :math:`q` is correct.
    
    * :math:`\texttt{isFlipped}_{q} \rightarrow E ~~` : true if the part in quadrant :math:`q` is flipped.
    
    * :math:`\texttt{isFaulty}_{q} \rightarrow F ~~` : true if the part in quadrant :math:`q` is faulty.

.. admonition:: Tray Points

  .. math::

    \texttt{pt}_{t} = \begin{cases}
    1, &\text{if} ~~ A \\
    0, &\text{otherwise}  \\
    \end{cases}
  
.. admonition:: Quadrant Points

  .. math::

    \texttt{pt}_q = \begin{cases}
    0, &\text{if} ~~ \lnot C \lor E \lor F \\
    3, &\text{if} ~~ D \\
    2, &\text{if} ~~ \lnot D \\
    \end{cases}

.. admonition:: Bonus Points

  .. math::

    \texttt{pt}_b = \begin{cases}
    n, &\text{if} ~~ \sum_{q}^{n}{\texttt{pt}_q} = n\times 3 \\
    0, &\text{otherwise} \\
    \end{cases}
   
.. admonition:: Extra Parts Penalty

  A penalty is only applied if more parts are on the tray than needed.

  .. math::

    \texttt{pn}_{ep} = \begin{cases}
    m - n, &\text{if} ~~ m>n \\
    0, &\text{otherwise} \\
    \end{cases}

.. admonition:: Wrong Tray Penalty

  .. math::

    \texttt{pn}_{t} = \begin{cases}
    1, &\text{if} ~~ \lnot A \\
    0, &\text{otherwise} \\
    \end{cases}

.. admonition:: Destination Multiplier

  .. math::

    \texttt{pm}_{d} = \begin{cases}
    1, &\text{if}\, B \\
    0, &\text{otherwise} \\
    \end{cases}

.. admonition:: Kitting Task Score
  :class: tip
  :name: task-score

  .. math::

    S_{k} = (\texttt{pt}_{t} + \sum_{q}^{n}{(\texttt{pt}_q)} + \texttt{pt}_b - \texttt{pn}_{ep} - \texttt{pn}_{t}) \times \texttt{pm}_{d}

  The task score cannot be negative, if the calculation is negative the score will be set as 0.


Assembly Task Score
===================

  * An assembly task has :math:`n` parts that need to be assembled into the insert.
  
  * For each task there is one Boolean condition:
    
    * :math:`\texttt{isCorrectStation} \rightarrow A ~~` : true if the assembly was done at the correct station (as1, as2, as3, or as4).
  
  * Each slot :math:`s` in the insert has the following Boolean conditions:
    
    * :math:`\texttt{isAssembled}_{s} \rightarrow B ~~` : true if the part in slot :math:`s` is reported as assembled. 
    
    * :math:`\texttt{isCorrectColor}_{s} \rightarrow C ~~` : true if the part in slot :math:`s` is of correct color.
    
    * :math:`\texttt{isCorrectPose}_{s} \rightarrow D ~~` : true if the part in slot :math:`s` has the correct pose.

.. admonition:: Slot Points

  .. math::

    \texttt{pt}_s = \begin{cases}
    0, &\text{if} ~~ \lnot B \\
    3, &\text{if} ~~ C \land D \\
    2, &\text{if} ~~ C \lor D \\
    1, &\text{if} ~~ \lnot C \land \lnot D\\
    \end{cases}

.. admonition:: Bonus Points

  .. math::

    \texttt{pt}_b = \begin{cases}
    n, &\text{if} ~~ \sum_{s}^{n}{\texttt{pt}_{s}} = n\times 3 \\
    0, &\text{otherwise} \\
    \end{cases}

.. admonition:: Station Multiplier

  .. math::

    \texttt{pm}_{s} = \begin{cases}
    1, &\text{if}\, A \\
    0, &\text{otherwise} \\
    \end{cases}

.. admonition:: Assembly Task Score
  :class: tip
  :name: task-score-assembly

  .. math::

    S_{a} = (\sum_{s}^{n}{\texttt{pt}_s} + \texttt{pt}_b) \times \texttt{pm}_{s}


Combined Task Score
===================

  * A combined task has :math:`n` parts that need to be gathered from the environment and assembled to the insert.
  
  * For each task there is one Boolean condition:

    * :math:`\texttt{isCorrectStation} \rightarrow A ~~` : true if the assembly was done at the correct station (as1, as2, as3, or as4).
  
  * Each slot :math:`s` in the insert has the following Boolean conditions:

    * :math:`\texttt{isAssembled}_{s} \rightarrow B ~~` : true if the part in slot :math:`s` is reported as assembled. 
    
    * :math:`\texttt{isCorrectColor}_{s} \rightarrow C ~~` : true if the part in slot :math:`s` is of correct color.
    
    * :math:`\texttt{isCorrectPose}_{s} \rightarrow D ~~` : true if the part in slot :math:`s` has the correct pose.

.. admonition:: Slot Points

  .. math::

    \texttt{pt}_s = \begin{cases}
    0, &\text{if} ~~ \lnot B \\
    5, &\text{if} ~~ C \land D \\
    4, &\text{if} ~~ C \lor D \\
    3, &\text{if} ~~ \lnot C \land \lnot D\\
    \end{cases}

.. admonition:: Bonus Points

  .. math::

    \texttt{pt}_b = \begin{cases}
    n, &\text{if} ~~ \sum_{s}^{n}{\texttt{pt}_{s}} = n\times 5 \\
    0, &\text{otherwise} \\
    \end{cases}

.. admonition:: Station Multiplier

  .. math::

    \texttt{pm}_{s} = \begin{cases}
    1, &\text{if}\, A \\
    0, &\text{otherwise} \\
    \end{cases}

.. admonition:: Combined Task Score
  :class: tip
  :name: task-score-combined

  .. math::

    S_{c} = (\sum_{s}^{n}{\texttt{pt}_s} + \texttt{pt}_b) \times \texttt{pm}_{s}


-----------
Trial Score
-----------

The trial score :math:`TS` combines the cost factor, efficiency factors and task scores into a single score for ranking the teams.

* For each order there is one Boolean condition:

  * :math:`\texttt{isPriorityOrder} \rightarrow A ~~` : true if the order is classified as a priority order

.. admonition:: Priority Multiplier

   .. math::

        \texttt{pm}_p = \begin{cases}
        3, &\text{if} ~~ A \\
        1, &\text{otherwise} \\
        \end{cases}


.. admonition:: Trial Score
  :class: caution
  :name: trial-score

  .. math::

    TS = CF \times \sum_{i=0}^{n}{(\texttt{pm}_p \times EF_i \times S_i)}
