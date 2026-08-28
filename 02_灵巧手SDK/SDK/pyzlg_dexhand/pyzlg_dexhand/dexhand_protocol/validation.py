"""Validation helpers for DexHand interface"""

import logging
from typing import Optional, Dict, Any, Tuple, Union
from .messages import LogLevel

logger = logging.getLogger(__name__)

class ValidationHelper:
    """Helper for validating input parameters"""
    
    @staticmethod
    def validate_parameter(param_name: str, 
                        value: Any, 
                        valid_range: Optional[Tuple[Union[int, float], Union[int, float]]] = None,
                        valid_values: Optional[set] = None,
                        instance_type: Optional[type] = None) -> Tuple[bool, Optional[str]]:
        """Validate a parameter against given criteria
        
        Args:
            param_name: Name of the parameter (for error messages)
            value: Value to validate
            valid_range: Optional tuple of (min, max) for range validation
            valid_values: Optional set of valid values
            instance_type: Optional type check
            
        Returns:
            Tuple of (is_valid, error_message)
        """
        # Check type if specified
        if instance_type is not None and not isinstance(value, instance_type):
            return False, f"Invalid {param_name}: {value}. Must be of type {instance_type.__name__}"
        
        # Check range if specified
        if valid_range is not None:
            min_val, max_val = valid_range
            if not (min_val <= value <= max_val):
                return False, f"Invalid {param_name}: {value}. Must be {min_val}-{max_val}"
        
        # Check valid values if specified
        if valid_values is not None and value not in valid_values:
            return False, f"Invalid {param_name}: {value}. Must be one of {valid_values}"
            
        return True, None
    
    @staticmethod
    def validate_parameters(params: Dict[str, Dict[str, Any]], log_level: Optional[LogLevel] = None) -> Tuple[bool, Optional[str]]:
        """Validate multiple parameters together
        
        Args:
            params: Dictionary mapping parameter names to validation criteria
                Each entry is a dict with keys:
                - 'value': The value to validate
                - 'valid_range': Optional tuple of (min, max) for range validation
                - 'valid_values': Optional set of valid values
                - 'instance_type': Optional type check
            log_level: Optional logging level for error messages
        
        Returns:
            Tuple of (all_valid, first_error_message)
        """
        for param_name, criteria in params.items():
            value = criteria.get('value')
            valid_range = criteria.get('valid_range')
            valid_values = criteria.get('valid_values')
            instance_type = criteria.get('instance_type')
            
            is_valid, error_msg = ValidationHelper.validate_parameter(
                param_name, value, valid_range, valid_values, instance_type
            )
            
            if not is_valid:
                if log_level is not None and log_level <= LogLevel.ERROR:
                    logger.error(error_msg)
                return False, error_msg
                
        return True, None