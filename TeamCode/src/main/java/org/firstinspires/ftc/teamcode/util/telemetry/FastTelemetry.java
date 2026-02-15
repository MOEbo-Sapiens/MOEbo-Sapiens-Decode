package org.firstinspires.ftc.teamcode.util.telemetry;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Predicate;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.robocol.TelemetryMessage;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

public class FastTelemetry implements Telemetry {

    protected class Value<T> {
        protected String format = null;
        protected Object[] formatArgs = null;
        protected Object value = null;
        protected Func<T> valueProducer = null;
        protected String composed = null;

        Value(String format, Object... formatArgs) {
            this.format = format;
            this.formatArgs = formatArgs;
        }

        Value(String format, Func<T> valueProducer) {
            this.format = format;
            this.valueProducer = valueProducer;
        }

        Value(Object value) {
            if ((value instanceof Double) || (value instanceof Float)) {
                this.value = decimalFormat.format(value);
            } else {
                this.value = value;
            }
        }

        Value(Func<T> valueProducer) {
            this.valueProducer = valueProducer;
        }

        boolean isProducer() {
            return this.valueProducer != null;
        }

        String getComposed(boolean recompose) {
            if (recompose || composed == null) {
                composed = compose();
            }
            return composed;
        }

        protected String compose() {
            if (format != null) {
                if (formatArgs != null) {
                    return String.format(format, formatArgs);
                }
                if (valueProducer != null) {
                    return String.format(format, valueProducer.value());
                }
            } else {
                if (value != null) {
                    return value.toString();
                }
                if (valueProducer != null) {
                    return valueProducer.value().toString();
                }
            }

            return "";
        }
    }

    protected interface Lineable {
        String getComposed(boolean recompose);
    }

    protected class LineableContainer implements Iterable<Lineable> {
        private final ArrayList<Lineable> list = new ArrayList<>();

        void boundedAddToList(int index, Lineable data) {
            if (list.size() < TelemetryMessage.cCountMax) {
                list.add(index, data);
            }
        }

        @Override
        public Iterator<Lineable> iterator() {
            synchronized (theLock) {
                return list.iterator();
            }
        }

        Line addLineAfter(Lineable prev, String lineCaption) {
            synchronized (theLock) {
                onAddData();

                LineImpl result = new LineImpl(lineCaption, this);
                int index = prev == null ? list.size() : list.indexOf(prev) + 1;
                boundedAddToList(index, result);
                return result;
            }
        }

        Item addItemAfter(Lineable prev, String caption, Value value) {
            synchronized (theLock) {
                onAddData();

                ItemImpl result = new ItemImpl(this, caption, value);
                int index = prev == null ? list.size() : list.indexOf(prev) + 1;
                boundedAddToList(index, result);
                return result;
            }
        }

        boolean isEmpty() {
            synchronized (theLock) {
                return list.isEmpty();
            }
        }

        boolean remove(Lineable lineable) {
            synchronized (theLock) {
                for (int i = 0; i < list.size(); i++) {
                    if (list.get(i) == lineable) {
                        list.remove(i);
                        return true;
                    }
                }
                return false;
            }
        }

        boolean removeAllRecurse(Predicate<ItemImpl> predicate) {
            synchronized (theLock) {
                boolean result = false;
                for (int i = 0; i < list.size();) {
                    Lineable cur = list.get(i);
                    if (cur instanceof LineImpl) {
                        LineImpl line = (LineImpl) cur;
                        line.lineables.removeAllRecurse(predicate);

                        if (line.lineables.isEmpty()) {
                            list.remove(i);
                            result = true;
                        } else {
                            i++;
                        }
                    } else if (cur instanceof ItemImpl) {
                        if (predicate.test((ItemImpl) cur)) {
                            list.remove(i);
                            result = true;
                        } else {
                            i++;
                        }
                    } else {
                        i++;
                    }
                }
                return result;
            }
        }
    }

    protected class ItemImpl implements Item, Lineable {
        final LineableContainer parent;
        String caption = null;
        Value value = null;
        Boolean retained = null;

        boolean showIfEmpty = true;

        ItemImpl(LineableContainer parent, String caption, Value value) {
            this.parent = parent;
            this.caption = caption;
            this.value = value;
            this.retained = null;
        }

        @Override
        public String getComposed(boolean recompose) {
            synchronized (theLock) {
                String composed = this.value.getComposed(recompose);

                if (!showIfEmpty && this.caption.trim().isEmpty() && composed.trim().isEmpty()) {
                    return "";
                }

                return String.format("%s%s%s", this.caption, getCaptionValueSeparator(), composed);
            }
        }

        @Override
        public String getCaption() {
            return this.caption;
        }

        @Override
        public Item setCaption(String caption) {
            this.caption = caption;
            return this;
        }

        @Override
        public boolean isRetained() {
            synchronized (theLock) {
                return this.retained != null ? this.retained : this.isProducer();
            }
        }

        @Override
        public Item setRetained(Boolean retained) {
            synchronized (theLock) {
                this.retained = retained;
                return this;
            }
        }

        boolean isProducer() {
            synchronized (theLock) {
                return this.value.isProducer();
            }
        }

        void internalSetValue(Value value) {
            synchronized (theLock) {
                this.value = value;
            }
        }

        @Override
        public Item setValue(String format, Object... args) {
            internalSetValue(new Value(format, args));
            return this;
        }

        @Override
        public Item setValue(Object value) {
            internalSetValue(new Value(value));
            return this;
        }

        @Override
        public <T> Item setValue(Func<T> valueProducer) {
            internalSetValue(new Value<>(valueProducer));
            return this;
        }

        @Override
        public <T> Item setValue(String format, Func<T> valueProducer) {
            internalSetValue(new Value<>(format, valueProducer));
            return this;
        }

        @Override
        public Item addData(String caption, String format, Object... args) {
            return parent.addItemAfter(this, caption, new Value(format, args));
        }

        @Override
        public Item addData(String caption, Object value) {
            return parent.addItemAfter(this, caption, new Value(value));
        }

        @Override
        public <T> Item addData(String caption, Func<T> valueProducer) {
            return parent.addItemAfter(this, caption, new Value<>(valueProducer));
        }

        @Override
        public <T> Item addData(String caption, String format, Func<T> valueProducer) {
            return parent.addItemAfter(this, caption, new Value<>(format, valueProducer));
        }
    }

    protected class LineImpl implements Line, Lineable {
        final LineableContainer parent;
        String lineCaption;
        LineableContainer lineables;

        LineImpl(String lineCaption, LineableContainer parent) {
            this.parent = parent;
            this.lineCaption = lineCaption;
            this.lineables = new LineableContainer();
        }

        @Override
        public String getComposed(boolean recompose) {
            StringBuilder result = new StringBuilder();
            result.append(this.lineCaption);
            boolean firstTime = true;
            for (Lineable lineable : lineables) {
                if (!firstTime) {
                    result.append(getItemSeparator());
                }
                result.append(lineable.getComposed(recompose));
                firstTime = false;
            }
            return result.toString();
        }

        @Override
        public Item addData(String caption, String format, Object... args) {
            return lineables.addItemAfter(null, caption, new Value(format, args));
        }

        @Override
        public Item addData(String caption, Object value) {
            return lineables.addItemAfter(null, caption, new Value(value));
        }

        @Override
        public <T> Item addData(String caption, Func<T> valueProducer) {
            return lineables.addItemAfter(null, caption, new Value<>(valueProducer));
        }

        @Override
        public <T> Item addData(String caption, String format, Func<T> valueProducer) {
            return lineables.addItemAfter(null, caption, new Value<>(format, valueProducer));
        }
    }

    protected class LogImpl implements Log {
        List<String> entries;
        int capacity;
        DisplayOrder displayOrder;
        boolean isDirty;

        LogImpl() {
            reset();
        }

        void markDirty() {
            this.isDirty = true;
        }

        void markClean() {
            this.isDirty = false;
        }

        boolean isDirty() {
            return this.isDirty;
        }

        Object getLock() {
            return FastTelemetry.this;
        }

        int size() {
            return entries.size();
        }

        String get(int index) {
            return entries.get(index);
        }

        void prune() {
            synchronized (getLock()) {
                while (this.entries.size() > this.capacity && this.entries.size() > 0) {
                    this.entries.remove(0);
                }
            }
        }

        void reset() {
            this.entries = new ArrayList<>();
            this.capacity = 9;
            this.isDirty = false;
            this.displayOrder = DisplayOrder.OLDEST_FIRST;
        }

        @Override
        public int getCapacity() {
            return this.capacity;
        }

        @Override
        public void setCapacity(int capacity) {
            synchronized (getLock()) {
                this.capacity = capacity;
                prune();
            }
        }

        @Override
        public DisplayOrder getDisplayOrder() {
            return this.displayOrder;
        }

        @Override
        public void setDisplayOrder(DisplayOrder displayOrder) {
            synchronized (getLock()) {
                this.displayOrder = displayOrder;
            }
        }

        @Override
        public void add(String format, Object... args) {
            synchronized (getLock()) {
                String datum = String.format(format, args);
                this.entries.add(datum);
                this.markDirty();
                this.prune();

                tryUpdate(UpdateReason.LOG);
            }
        }

        @Override
        public void add(String entry) {
            this.add("%s", entry);
        }

        @Override
        public void clear() {
            synchronized (getLock()) {
                this.entries.clear();
                this.markDirty();
            }
        }
    }

    protected final Object theLock = new Object();
    protected LineableContainer lines;
    protected List<String> composedLines;
    protected List<Runnable> actions;
    protected LogImpl log;
    protected ElapsedTime transmissionTimer;
    protected boolean isDirty;
    protected boolean clearOnAdd;
    protected boolean isAutoClear;
    protected int msTransmissionInterval;
    protected String captionValueSeparator;
    protected String itemSeparator;
    protected DecimalFormat decimalFormat = new DecimalFormat("0.####");
    protected final Telemetry delegate;

    public Item errItem;
    public Item infoItem;

    public FastTelemetry(Telemetry delegate) {
        this.delegate = delegate;
        this.log = new LogImpl();
        resetTelemetryForOpMode();
    }

    public void resetTelemetryForOpMode() {
        this.lines = new LineableContainer();
        this.composedLines = new ArrayList<>();
        this.actions = new LinkedList<>();
        log.reset();
        this.transmissionTimer = new ElapsedTime();
        this.isDirty = false;
        this.clearOnAdd = false;
        this.isAutoClear = true;
        this.msTransmissionInterval = 250;
        this.captionValueSeparator = " : ";
        this.itemSeparator = " | ";

        errItem = addData("", "").setRetained(true);
        ((ItemImpl) errItem).showIfEmpty = false;

        infoItem = addData("", "").setRetained(true);
        ((ItemImpl) infoItem).showIfEmpty = false;
    }

    void markDirty() {
        this.isDirty = true;
    }

    void markClean() {
        this.isDirty = false;
    }

    boolean isDirty() {
        return this.isDirty;
    }

    public void setNumDecimalPlaces(int minDecimalPlaces, int maxDecimalPlaces) {
        decimalFormat.setMinimumFractionDigits(minDecimalPlaces);
        decimalFormat.setMaximumFractionDigits(maxDecimalPlaces);
    }

    @Override
    public boolean update() {
        return tryUpdate(UpdateReason.USER);
    }

    public boolean tryUpdateIfDirty() {
        return tryUpdate(UpdateReason.IFDIRTY);
    }

    protected enum UpdateReason { USER, LOG, IFDIRTY }

    protected boolean tryUpdate(UpdateReason updateReason) {
        synchronized (theLock) {
            boolean result = false;

            boolean intervalElapsed = this.transmissionTimer.milliseconds() > msTransmissionInterval;

            boolean wantToTransmit = updateReason == UpdateReason.USER
                    || updateReason == UpdateReason.LOG
                    || (updateReason == UpdateReason.IFDIRTY && (isDirty() || log.isDirty()));

            boolean recompose = updateReason == UpdateReason.USER
                    || isDirty();

            if (intervalElapsed && wantToTransmit) {
                for (Runnable action : this.actions) {
                    action.run();
                }

                this.saveToTransmitter(recompose);

                this.log.markClean();
                markClean();

                this.transmissionTimer.reset();
                result = true;
            } else if (updateReason == UpdateReason.USER) {
                this.markDirty();
            }

            if (updateReason == UpdateReason.USER) {
                this.clearOnAdd = isAutoClear();
            }

            return result;
        }
    }

    protected void saveToTransmitter(boolean recompose) {
        if (recompose) {
            this.composedLines = new ArrayList<>();
            for (Lineable lineable : this.lines) {
                this.composedLines.add(lineable.getComposed(recompose));
            }
        }

        List<String> outputLines = new ArrayList<>();
        outputLines.addAll(this.composedLines);

        int size = this.log.size();
        for (int i = 0; i < size; i++) {
            String s = this.log.getDisplayOrder() == Log.DisplayOrder.OLDEST_FIRST
                    ? this.log.get(i)
                    : this.log.get(size - 1 - i);
            outputLines.add(s);
        }

        delegate.clearAll();
        for (String line : outputLines) {
            if (!line.isEmpty()) {
                delegate.addLine(line);
            }
        }
        delegate.update();
    }

    @Override
    public Log log() {
        return this.log;
    }

    @Override
    public boolean isAutoClear() {
        return this.isAutoClear;
    }

    @Override
    public void setAutoClear(boolean autoClear) {
        synchronized (theLock) {
            this.isAutoClear = autoClear;
        }
    }

    @Override
    public int getMsTransmissionInterval() {
        return this.msTransmissionInterval;
    }

    @Override
    public void setMsTransmissionInterval(int msTransmissionInterval) {
        synchronized (theLock) {
            this.msTransmissionInterval = msTransmissionInterval;
        }
    }

    @Override
    public String getItemSeparator() {
        return this.itemSeparator;
    }

    @Override
    public void setItemSeparator(String itemSeparator) {
        synchronized (theLock) {
            this.itemSeparator = itemSeparator;
        }
    }

    @Override
    public String getCaptionValueSeparator() {
        return this.captionValueSeparator;
    }

    @Override
    public void setCaptionValueSeparator(String captionValueSeparator) {
        synchronized (theLock) {
            this.captionValueSeparator = captionValueSeparator;
        }
    }

    @Override
    public void setDisplayFormat(DisplayFormat displayFormat) {
        delegate.setDisplayFormat(displayFormat);
    }

    @Override
    public Object addAction(Runnable action) {
        synchronized (theLock) {
            this.actions.add(action);
            return action;
        }
    }

    @Override
    public boolean removeAction(Object token) {
        synchronized (theLock) {
            return this.actions.remove((Runnable) token);
        }
    }

    @Override
    public Item addData(String caption, String format, Object... args) {
        return this.lines.addItemAfter(null, caption, new Value(format, args));
    }

    @Override
    public Item addData(String caption, Object value) {
        return this.lines.addItemAfter(null, caption, new Value(value));
    }

    @Override
    public <T> Item addData(String caption, Func<T> valueProducer) {
        return this.lines.addItemAfter(null, caption, new Value<>(valueProducer));
    }

    @Override
    public <T> Item addData(String caption, String format, Func<T> valueProducer) {
        return this.lines.addItemAfter(null, caption, new Value<>(format, valueProducer));
    }

    @Override
    public Line addLine() {
        return this.lines.addLineAfter(null, "");
    }

    @Override
    public Line addLine(String lineCaption) {
        return this.lines.addLineAfter(null, lineCaption);
    }

    @Override
    public boolean removeItem(Item item) {
        if (item instanceof ItemImpl) {
            ItemImpl itemImpl = (ItemImpl) item;
            return itemImpl.parent.remove(itemImpl);
        }
        return false;
    }

    @Override
    public boolean removeLine(Line line) {
        if (line instanceof LineImpl) {
            LineImpl lineImpl = (LineImpl) line;
            return lineImpl.parent.remove(lineImpl);
        }
        return false;
    }

    protected void onAddData() {
        if (this.clearOnAdd) {
            clear();
            this.clearOnAdd = false;
        }

        markClean();
    }

    @Override
    public void clear() {
        synchronized (theLock) {
            this.clearOnAdd = false;
            markClean();
            this.lines.removeAllRecurse(new Predicate<ItemImpl>() {
                @Override
                public boolean test(ItemImpl item) {
                    return !item.isRetained();
                }
            });
        }
    }

    @Override
    public void clearAll() {
        synchronized (theLock) {
            this.clearOnAdd = false;
            markClean();
            this.actions.clear();
            this.lines.removeAllRecurse(new Predicate<ItemImpl>() {
                @Override
                public boolean test(ItemImpl item) {
                    return true;
                }
            });
        }
    }

    @Override
    public void speak(String text) {
        speak(text, null, null);
    }

    @Override
    public void speak(String text, String languageCode, String countryCode) {
        delegate.speak(text, languageCode, countryCode);
    }
}
