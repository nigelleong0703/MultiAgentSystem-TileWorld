package tileworld.agent;

import java.util.Arrays;

// Hungarian algorithm implementation
public class HungarianAlgorithm {
    private double[][] costMatrix;
    private int rows, cols, dim;
    private double[] labelByWorker, labelByJob;
    private int[] minSlackWorkerByJob;
    private double[] minSlackValueByJob;
    private int[] matchJobByWorker, matchWorkerByJob;
    private int[] parentWorkerByCommittedJob;
    private boolean[] committedWorkers;

    public HungarianAlgorithm(double[][] costMatrix) {
        this.dim = Math.max(costMatrix.length, costMatrix[0].length);
        this.rows = costMatrix.length;
        this.cols = costMatrix[0].length;
        this.costMatrix = new double[this.dim][this.dim];
        for (int w = 0; w < this.dim; w++) {
            if (w < costMatrix.length) {
                if (costMatrix[w].length != this.cols) {
                    throw new IllegalArgumentException("Irregular cost matrix");
                }
                this.costMatrix[w] = Arrays.copyOf(costMatrix[w], this.dim);
            } else {
                this.costMatrix[w] = new double[this.dim];
            }
        }
        labelByWorker = new double[this.dim];
        labelByJob = new double[this.dim];
        minSlackWorkerByJob = new int[this.dim];
        minSlackValueByJob = new double[this.dim];
        committedWorkers = new boolean[this.dim];
        parentWorkerByCommittedJob = new int[this.dim];
        matchJobByWorker = new int[this.dim];
        Arrays.fill(matchJobByWorker, -1);
        matchWorkerByJob = new int[this.dim];
        Arrays.fill(matchWorkerByJob, -1);
    }

    protected void execute2() {
        // Initialize
        reduce();
        computeInitialFeasibleSolution();
        greedyMatch();

        int w = fetchUnmatchedWorker();
        while (w < dim) {
            initializePhase(w);
            executePhase();
            w = fetchUnmatchedWorker();
        }
    }

    protected int fetchUnmatchedWorker() {
        int w;
        for (w = 0; w < dim; w++) {
            if (matchJobByWorker[w] == -1) {
                break;
            }
        }
        return w;
    }

    protected void reduce() {
        for (int w = 0; w < dim; w++) {
            double min = Double.POSITIVE_INFINITY;
            for (int j = 0; j < dim; j++) {
                if (costMatrix[w][j] < min) {
                    min = costMatrix[w][j];
                }
            }
            for (int j = 0; j < dim; j++) {
                costMatrix[w][j] -= min;
            }
        }
        double[] min = new double[dim];
        for (int j = 0; j < dim; j++) {
            min[j] = Double.POSITIVE_INFINITY;
        }
        for (int w = 0; w < dim; w++) {
            for (int j = 0; j < dim; j++) {
                if (costMatrix[w][j] < min[j]) {
                    min[j] = costMatrix[w][j];
                }
            }
        }
        for (int j = 0; j < dim; j++) {
            for (int w = 0; w < dim; w++) {
                costMatrix[w][j] -= min[j];
            }
        }
    }

    protected void computeInitialFeasibleSolution() {
        for (int j = 0; j < dim; j++) {
            labelByJob[j] = Double.POSITIVE_INFINITY;
        }
        for (int w = 0; w < dim; w++) {
            for (int j = 0; j < dim; j++) {
                if (costMatrix[w][j] < labelByJob[j]) {
                    labelByJob[j] = costMatrix[w][j];
                }
            }
        }
    }

    protected void greedyMatch() {
        for (int w = 0; w < dim; w++) {
            for (int j = 0; j < dim; j++) {
                if (matchJobByWorker[w] == -1 && matchWorkerByJob[j] == -1 && costMatrix[w][j] - labelByWorker[w] - labelByJob[j] == 0) {
                    match(w, j);
                }
            }
        }
    }

    protected void initializePhase(int w) {
        Arrays.fill(committedWorkers, false);
        Arrays.fill(parentWorkerByCommittedJob, -1);
        committedWorkers[w] = true;
        for (int j = 0; j < dim; j++) {
            minSlackValueByJob[j] = costMatrix[w][j] - labelByWorker[w] - labelByJob[j];
            minSlackWorkerByJob[j] = w;
        }
    }

    protected void executePhase() {
        while (true) {
            int minSlackJob = -1;
            double minSlackValue = Double.POSITIVE_INFINITY;
            for (int j = 0; j < dim; j++) {
                if (parentWorkerByCommittedJob[j] == -1 && minSlackValueByJob[j] < minSlackValue) {
                    minSlackValue = minSlackValueByJob[j];
                    minSlackJob = j;
                }
            }
            if (minSlackValue > 0) {
                updateLabeling(minSlackValue);
            }
            parentWorkerByCommittedJob[minSlackJob] = minSlackWorkerByJob[minSlackJob];
            if (matchWorkerByJob[minSlackJob] == -1) {
                /*
                 * An unmatched job was found, so augment the matching and break out of this phase.
                 */
                int committedJob = minSlackJob;
                int parentWorker = parentWorkerByCommittedJob[committedJob];
                while (true) {
                    int temp = matchJobByWorker[parentWorker];
                    match(parentWorker, committedJob);
                    committedJob = temp;
                    if (committedJob == -1) {
                        break;
                    }
                    parentWorker = parentWorkerByCommittedJob[committedJob];
                }
                return;
            } else {
                /*
                 * Match the slack job to the slack worker. Mark the worker as committed.
                 * Each worker is committed at most once during an augmenting path calculation, so this is safe.
                 */
                int worker = matchWorkerByJob[minSlackJob];
                committedWorkers[worker] = true;
                for (int j = 0; j < dim; j++) {
                    if (parentWorkerByCommittedJob[j] == -1) {
                        double slack = costMatrix[worker][j] - labelByWorker[worker] - labelByJob[j];
                        if (slack < minSlackValueByJob[j]) {
                            minSlackValueByJob[j] = slack;
                            minSlackWorkerByJob[j] = worker;
                        }
                    }
                }
            }
        }
    }

    protected void updateLabeling(double slack) {
        for (int w = 0; w < dim; w++) {
            if (committedWorkers[w]) {
                labelByWorker[w] += slack;
            }
        }
        for (int j = 0; j < dim; j++) {
            if (parentWorkerByCommittedJob[j] != -1) {
                labelByJob[j] -= slack;
            } else {
                minSlackValueByJob[j] -= slack;
            }
        }
    }

    protected void match(int w, int j) {
        matchJobByWorker[w] = j;
        matchWorkerByJob[j] = w;
    }

    public int[] execute() {
        execute2();
        return Arrays.copyOf(matchJobByWorker, rows);
    }
}