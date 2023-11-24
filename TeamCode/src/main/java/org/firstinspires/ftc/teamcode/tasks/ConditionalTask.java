package org.firstinspires.ftc.teamcode.tasks;


import java.util.function.Supplier;

public class ConditionalTask extends CompoundTask {
    Supplier<Boolean> condition;
    public int currentTask = 0;

    public ConditionalTask(Supplier<Boolean> condition, Task... children) {
        super(children);
        this.condition = condition;
    }

    @Override
    public void tick() {
        if (!condition.get()) {
            state = State.FINISHED;
            return;
        }
        if (children[currentTask].isFinished()) {
            currentTask++;
            if (currentTask >= children.length) {
                state = State.FINISHED;
                currentTask = 0;
                for (Task child : children) child.state = State.DEFAULT;
                return;
            }
            children[currentTask].start(this.context);
        } else if (children[currentTask].isRunning()) {
            children[currentTask].tick();
        }
    }

    @Override
    public void run() {
        System.out.println("fjffksjksjfsk");
        System.out.println(condition.get());
        if (!condition.get()) {
            state = State.FINISHED;
            return;
        }
        if (children.length == 0) {
            state = State.FINISHED;
            return;
        }
        children[0].start(context);
    }
}
