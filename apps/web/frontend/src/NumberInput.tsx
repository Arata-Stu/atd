import type { ComponentProps } from 'react';

type Props = Omit<ComponentProps<'input'>, 'type' | 'value' | 'defaultValue' | 'onChange'> & {
  value: number;
  onValueChange: (value: number) => void;
};

export function NumberInput({ value, onValueChange, ...props }: Props) {
  return <input {...props} type="number" value={Number.isFinite(value) ? value : ''}
    onChange={event => onValueChange(event.currentTarget.valueAsNumber)}/>;
}
